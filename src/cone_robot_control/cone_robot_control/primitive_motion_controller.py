#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: High-Level Motion Primitive Controller
==============================================================================
Target Platform: Raspberry Pi 5 / ROS 2 Jazzy
Hardware: Cytron MDD10, MikroE BNO08x IMU, YDLIDAR T-mini Plus, rf2o laser odometry

Description:
    Implements a high-level motion execution engine providing:
      1. ROS 2 Action Server: `/execute_motion` (cone_robot_interfaces/action/ExecuteMotion)
         - Supports DRIVE_STRAIGHT, ROTATE_IN_PLACE, DRIVE_ARC, and STOP.
         - Real-time preemption, cancellation, and progress feedback.
      2. Deterministic S-Curve / Trapezoidal Trajectory Profiling:
         - Eliminates track-slip jerk at motion start (continuous curvature).
      3. Dual-Rate Feedback Controller:
         - Fast 50 Hz IMU heading / yaw-rate PI loop.
         - 6 Hz LiDAR planar odometry distance tracking.
      4. Stuck / Stall Watchdog:
         - Detects track stall when commanded effort produces no motion.
      5. Backward Compatibility:
         - Subscribes to `/cmd_step` (Vector3: cm, deg) for legacy step testing.
         - Subscribes to `/cmd_primitive` (Vector3: dist_m, 0, delta_yaw_rad).

Publishes:
    - `/cmd_vel` (geometry_msgs/msg/Twist): Continuous velocity to mdd10_motor_controller.
    - `/motion_status` (std_msgs/msg/String): Current state machine status.
    - `/robot/diagnostics` (std_msgs/msg/String): SoC temperature & voltage.

Author: ConeRobot Team
License: MIT
==============================================================================
"""

import math
import os
import time
import json
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, BatteryState
from std_msgs.msg import Float32, String

# Try importing action interfaces
_ACTION_IMPORT_ERROR = None
try:
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from cone_robot_interfaces.action import ExecuteMotion
    HAS_ACTION_MSGS = True
except ImportError as e:
    HAS_ACTION_MSGS = False
    _ACTION_IMPORT_ERROR = str(e)

# Try importing nav_msgs
try:
    from nav_msgs.msg import Odometry
    NAV_MSGS_AVAILABLE = True
except ImportError:
    NAV_MSGS_AVAILABLE = False
    Odometry = None

try:
    from .motion_profiler import MotionPrimitiveProfiler
except (ImportError, ValueError):
    try:
        from cone_robot_control.motion_profiler import MotionPrimitiveProfiler
    except ImportError:
        import motion_profiler
        MotionPrimitiveProfiler = motion_profiler.MotionPrimitiveProfiler


def normalize_angle_rad(rad: float) -> float:
    """Normalize angle to [-pi, pi] radians."""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def normalize_angle_deg(deg: float) -> float:
    """Normalize angle to [0.0, 360.0) degrees."""
    return deg % 360.0


def shortest_angular_diff_rad(target_rad: float, current_rad: float) -> float:
    """
    Returns shortest signed angular difference (target - current) in radians.
    Result in [-pi, pi]. Positive = turn left (CCW).
    """
    diff = (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi
    return diff


def shortest_angular_diff_deg(target_deg: float, current_deg: float) -> float:
    """
    Returns shortest signed angular difference (target - current) in degrees.
    Result in [-180, 180]. Positive = turn left (CCW).
    """
    diff = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return diff


def gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compute flat-Earth metric distance in meters between two WGS84 GPS coordinates.
    Accurate for local ranges (< 1 km).
    """
    earth_radius_m = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    x = d_lon * math.cos(lat_avg)
    y = d_lat
    return math.sqrt(x * x + y * y) * earth_radius_m


class ControllerState:
    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    CANCELING = "CANCELING"
    COMPLETED = "COMPLETED"
    STUCK = "STUCK"


class PrimitiveMotionController(Node):
    """
    Executes high-level motion primitives via closed-loop IMU and LiDAR odometry feedback.
    """

    def __init__(self) -> None:
        super().__init__('primitive_motion_controller')
        self.cb_group = ReentrantCallbackGroup()

        # ----------------------------------------------------------------------
        # ROS 2 Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('control_rate_hz', 50.0)         # Inner loop rate (matches IMU)
        self.declare_parameter('feedback_rate_hz', 10.0)        # Action feedback publication rate

        # Default Speed & Acceleration Limits
        self.declare_parameter('default_linear_speed', 0.25)    # Cruising linear speed (m/s)
        self.declare_parameter('default_angular_speed', 1.0)    # Cruising angular speed (rad/s)
        self.declare_parameter('max_linear_speed', 0.60)        # Hard limit on linear speed (m/s)
        self.declare_parameter('max_angular_speed', 2.0)        # Hard limit on angular speed (rad/s)
        self.declare_parameter('max_linear_accel', 0.40)        # Linear acceleration (m/s^2)
        self.declare_parameter('max_angular_accel', 1.20)       # Angular acceleration (rad/s^2)

        # Minimum Speeds (to overcome static track friction)
        self.declare_parameter('min_linear_speed', 0.08)        # Minimum speed to prevent track stall
        self.declare_parameter('min_angular_speed', 0.70)       # Assertive turn speed (matches step controller)

        # Skid-Steering Kinematic Parameters
        self.declare_parameter('wheel_track_geometric', 0.290)  # Physical track width in meters
        self.declare_parameter('wheel_track_effective', 0.380)  # Effective track width (L_eff) for slip

        # Closed-Loop Feedback Gains
        self.declare_parameter('yaw_kp', 1.2)                   # Proportional gain for heading correction
        self.declare_parameter('yaw_ki', 0.15)                  # Integral gain for heading trimming
        self.declare_parameter('yaw_tolerance_rad', math.radians(1.8)) # Target yaw tolerance (~1.8 deg)
        self.declare_parameter('turn_settle_time_s', 0.15)      # Standstill settle duration (seconds)
        self.declare_parameter('distance_tolerance_m', 0.03)    # Distance tolerance (3 cm)

        # Stuck & Safety Watchdog
        self.declare_parameter('stuck_detect_time_s', 1.5)      # Time threshold to trigger stuck abort (1.5s)
        self.declare_parameter('distance_source', 'auto')       # 'auto', 'odom', 'gps', 'time'
        self.declare_parameter('gps_require_rtk', True)         # Only use GPS if RTK Fix/Float is active

        # Read Parameters
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.feedback_rate_hz = float(self.get_parameter('feedback_rate_hz').value)
        self.default_linear_speed = float(self.get_parameter('default_linear_speed').value)
        self.default_angular_speed = float(self.get_parameter('default_angular_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)
        self.effective_track = float(self.get_parameter('wheel_track_effective').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_ki = float(self.get_parameter('yaw_ki').value)
        self.yaw_tolerance_rad = float(self.get_parameter('yaw_tolerance_rad').value)
        self.turn_settle_time_s = float(self.get_parameter('turn_settle_time_s').value)
        self.distance_tolerance_m = float(self.get_parameter('distance_tolerance_m').value)
        self.stuck_detect_time_s = float(self.get_parameter('stuck_detect_time_s').value)
        self.distance_source = self.get_parameter('distance_source').value
        self.gps_require_rtk = bool(self.get_parameter('gps_require_rtk').value)

        # ----------------------------------------------------------------------
        # Profiler & Trajectory Engine
        # ----------------------------------------------------------------------
        self.profiler = MotionPrimitiveProfiler(
            default_linear_speed=self.default_linear_speed,
            default_angular_speed=self.default_angular_speed,
            max_linear_accel=self.max_linear_accel,
            max_angular_accel=self.max_angular_accel,
        )

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/motion_status', 10)
        self.diag_pub = self.create_publisher(String, '/robot/diagnostics', 10)

        # Legacy / Compatibility Subscriptions
        self.cmd_step_sub = self.create_subscription(
            Vector3, '/cmd_step', self._cmd_step_callback, 10, callback_group=self.cb_group
        )
        self.cmd_prim_sub = self.create_subscription(
            Vector3, '/cmd_primitive', self._cmd_primitive_callback, 10, callback_group=self.cb_group
        )

        # Sensor Subscriptions
        self.heading_sub = self.create_subscription(
            Float32, '/imu/heading', self._heading_callback, 10, callback_group=self.cb_group
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_data_callback, 10, callback_group=self.cb_group
        )
        if NAV_MSGS_AVAILABLE:
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self._odom_callback, 10, callback_group=self.cb_group
            )
        else:
            self.odom_sub = None
        self.fix_sub = self.create_subscription(
            NavSatFix, '/fix', self._fix_callback, 10, callback_group=self.cb_group
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self._battery_callback, 10, callback_group=self.cb_group
        )

        # ----------------------------------------------------------------------
        # Action Server
        # ----------------------------------------------------------------------
        self.action_server = None
        if HAS_ACTION_MSGS:
            try:
                self.action_server = ActionServer(
                    self,
                    ExecuteMotion,
                    'execute_motion',
                    execute_callback=self._execute_action_callback,
                    cancel_callback=self._cancel_action_callback,
                    callback_group=self.cb_group,
                )
                self.get_logger().info("Action Server [execute_motion] initialized successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Action Server: {e}")
        else:
            self.get_logger().warn(
                f"cone_robot_interfaces action not available ({_ACTION_IMPORT_ERROR})! Running in topic-only mode (/cmd_step, /cmd_primitive)."
            )

        # ----------------------------------------------------------------------
        # Internal State Variables
        # ----------------------------------------------------------------------
        self.state = ControllerState.IDLE
        self.current_heading_rad: Optional[float] = None
        self.unrolled_yaw_rad: float = 0.0
        self.start_unrolled_yaw: float = 0.0
        self.current_yaw_rate: float = 0.0
        self.last_heading_time: float = 0.0

        # Odometry State
        self.current_odom_pos: Optional[Tuple[float, float]] = None
        self.start_odom_pos: Optional[Tuple[float, float]] = None
        self.last_odom_time: float = 0.0

        # GPS State (RTK Position & Incremental Path Odometer)
        self.current_gps_coords: Optional[Tuple[float, float]] = None
        self.start_gps_coords: Optional[Tuple[float, float]] = None
        self.last_gps_accum_coords: Optional[Tuple[float, float]] = None
        self.accumulated_gps_dist_m: float = 0.0
        self.gps_status: int = NavSatStatus.STATUS_NO_FIX
        self.last_gps_time: float = 0.0

        # Battery State
        self.battery_voltage: float = 12.0

        # Active Motion Goal
        self.active_motion_type = "NONE"
        self.target_dist_m: float = 0.0
        self.target_yaw_rad: float = 0.0
        self.start_heading_rad: float = 0.0
        self.motion_start_time: Optional[float] = None
        self.yaw_integral: float = 0.0
        self.turn_settle_start: Optional[float] = None

        # Stuck Detection & Active Command Tracking Variables
        self.stuck_check_start: Optional[float] = None
        self.last_moved_time: float = 0.0
        self.last_dist_m: float = 0.0
        self.last_yaw_rad: float = 0.0
        self.last_cmd_v: float = 0.0
        self.last_cmd_w: float = 0.0
        self.current_max_v: float = self.max_linear_speed
        self.current_max_w: float = self.max_angular_speed

        # Action Execution Reference
        self._current_goal_handle = None
        self._cancel_requested: bool = False

        # Timers
        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.diag_timer = self.create_timer(2.0, self._publish_diagnostics)

        self.get_logger().info("==================================================")
        self.get_logger().info(" High-Level Primitive Motion Controller Ready")
        self.get_logger().info(f" Control Rate   : {self.control_rate_hz} Hz")
        self.get_logger().info(f" Effective Track: {self.effective_track:.3f} m")
        self.get_logger().info("==================================================")

    # --------------------------------------------------------------------------
    # Sensor Callbacks
    # --------------------------------------------------------------------------
    def _heading_callback(self, msg: Float32) -> None:
        """Converts [0, 360] degrees heading into standard ROS [-pi, pi] radians and unrolls continuous yaw."""
        # Note: BNO08x heading in this repo is 0-360 deg clockwise/compass or CCW
        rad = math.radians(normalize_angle_deg(msg.data))
        norm_rad = normalize_angle_rad(rad)

        if self.current_heading_rad is not None:
            diff = shortest_angular_diff_rad(norm_rad, self.current_heading_rad)
            self.unrolled_yaw_rad += diff
        else:
            self.unrolled_yaw_rad = norm_rad

        self.current_heading_rad = norm_rad
        self.last_heading_time = time.time()

    def _imu_data_callback(self, msg: Imu) -> None:
        self.current_yaw_rate = msg.angular_velocity.z

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_odom_time = time.time()

    def _fix_callback(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            curr_lat = msg.latitude
            curr_lon = msg.longitude
            self.current_gps_coords = (curr_lat, curr_lon)
            self.gps_status = msg.status.status
            self.last_gps_time = time.time()

            # Incrementally accumulate path distance during active motion
            if self.state == ControllerState.EXECUTING and self.last_gps_accum_coords is not None:
                is_rtk_ok = (not self.gps_require_rtk) or (
                    self.gps_status in [NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.STATUS_SBAS_FIX]
                )
                if is_rtk_ok:
                    step_m = gps_distance_m(
                        self.last_gps_accum_coords[0], self.last_gps_accum_coords[1],
                        curr_lat, curr_lon
                    )
                    # Filter out static noise (< 3mm) and reject multi-path glitch jumps (> 50cm in 100ms)
                    if step_m > 0.50:
                        self.last_gps_accum_coords = (curr_lat, curr_lon)
                    elif step_m >= 0.003:
                        self.accumulated_gps_dist_m += step_m
                        self.last_gps_accum_coords = (curr_lat, curr_lon)

    def _battery_callback(self, msg: BatteryState) -> None:
        if msg.voltage > 5.0:
            self.battery_voltage = msg.voltage

    # --------------------------------------------------------------------------
    # Legacy & Quick-Test Topic Callbacks
    # --------------------------------------------------------------------------
    def _cmd_step_callback(self, msg: Vector3) -> None:
        """
        Receives legacy /cmd_step:
          msg.x = distance in cm (+ forward, - reverse)
          msg.z = turn in degrees (+ CCW left, - CW right)
        """
        dist_m = msg.x / 100.0
        yaw_rad = math.radians(msg.z)

        if abs(dist_m) > 0.005 and abs(yaw_rad) > 0.02:
            self._start_motion("ARC", dist_m, yaw_rad)
        elif abs(yaw_rad) > 0.02:
            self._start_motion("ROTATE", 0.0, yaw_rad)
        elif abs(dist_m) > 0.005:
            self._start_motion("STRAIGHT", dist_m, 0.0)
        else:
            self._stop_motion()

    def _cmd_primitive_callback(self, msg: Vector3) -> None:
        """
        Receives /cmd_primitive:
          msg.x = distance in meters
          msg.z = delta yaw in radians
        """
        dist_m = msg.x
        yaw_rad = msg.z

        if abs(dist_m) > 0.005 and abs(yaw_rad) > 0.01:
            self._start_motion("ARC", dist_m, yaw_rad)
        elif abs(yaw_rad) > 0.01:
            self._start_motion("ROTATE", 0.0, yaw_rad)
        elif abs(dist_m) > 0.005:
            self._start_motion("STRAIGHT", dist_m, 0.0)
        else:
            self._stop_motion()

    # --------------------------------------------------------------------------
    # ROS 2 Action Server Callbacks
    # --------------------------------------------------------------------------
    def _goal_action_callback(self, goal_request):
        self.get_logger().info(f"[ACTION] Received Goal: Type={goal_request.motion_type}")
        return GoalResponse.ACCEPT

    def _cancel_action_callback(self, goal_handle):
        self.get_logger().info("[ACTION] Preemption / Cancel requested by client.")
        self._cancel_requested = True
        self._stop_motion()
        return CancelResponse.ACCEPT

    def _execute_action_callback(self, goal_handle):
        self._current_goal_handle = goal_handle
        self._cancel_requested = False
        goal = goal_handle.request

        # Map Action Constants to internal primitive type
        # TYPE_DRIVE_STRAIGHT=0, TYPE_ROTATE_IN_PLACE=1, TYPE_DRIVE_ARC=2, TYPE_STOP=3
        if goal.motion_type == 0:
            m_type = "STRAIGHT"
        elif goal.motion_type == 1:
            m_type = "ROTATE"
        elif goal.motion_type == 2:
            m_type = "ARC"
        elif goal.motion_type == 3:
            m_type = "STOP"
        else:
            goal_handle.abort()
            result = ExecuteMotion.Result()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_INVALID_GOAL
            result.message = f"Unknown motion_type {goal.motion_type}"
            return result

        if m_type == "STOP":
            self._stop_motion()
            goal_handle.succeed()
            result = ExecuteMotion.Result()
            result.success = True
            result.error_code = ExecuteMotion.Result.ERROR_NONE
            result.message = "Stop complete"
            return result

        # Start Motion
        self._start_motion(
            m_type,
            goal.distance,
            goal.delta_yaw,
            max_v=goal.max_velocity if goal.max_velocity > 0 else None,
            max_w=goal.max_yaw_rate if goal.max_yaw_rate > 0 else None,
        )

        feedback = ExecuteMotion.Feedback()
        rate = self.create_rate(self.feedback_rate_hz)

        # Wait loop until motion finishes, aborts, or cancels
        while rclpy.ok() and self.state == ControllerState.EXECUTING:
            if self._cancel_requested or goal_handle.is_cancel_requested:
                self._stop_motion()
                self._set_state(ControllerState.CANCELING)
                goal_handle.canceled()
                result = ExecuteMotion.Result()
                result.success = False
                result.error_code = ExecuteMotion.Result.ERROR_CANCELED
                result.message = "Motion canceled by client"
                result.actual_distance = float(self._get_measured_distance_m())
                result.actual_yaw = float(self._get_measured_yaw_rad())
                self._current_goal_handle = None
                self._set_state(ControllerState.IDLE)
                return result

            # Publish Feedback
            measured_dist = self._get_measured_distance_m()
            measured_yaw = self._get_measured_yaw_rad()
            total_target = max(abs(self.target_dist_m), abs(self.target_yaw_rad))
            total_actual = max(abs(measured_dist), abs(measured_yaw))

            feedback.progress_ratio = min(1.0, max(0.0, total_actual / max(1e-4, total_target)))
            feedback.distance_remaining = max(0.0, abs(self.target_dist_m) - abs(measured_dist))
            feedback.yaw_remaining = max(0.0, abs(self.target_yaw_rad) - abs(measured_yaw))
            feedback.current_velocity = float(self.profiler.sample(time.time() - self.motion_start_time)[0])
            feedback.current_yaw_rate = float(self.current_yaw_rate)
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Evaluate final outcome
        result = ExecuteMotion.Result()
        result.actual_distance = float(self._get_measured_distance_m())
        result.actual_yaw = float(self._get_measured_yaw_rad())

        if self.state == ControllerState.STUCK:
            goal_handle.abort()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_STUCK
            result.message = "Robot stall / stuck detected by watchdog"
        else:
            goal_handle.succeed()
            result.success = True
            result.error_code = ExecuteMotion.Result.ERROR_NONE
            result.message = "Motion executed successfully"

        self._current_goal_handle = None
        self._set_state(ControllerState.IDLE)
        return result

    # --------------------------------------------------------------------------
    # Motion Initialization & State Management
    # --------------------------------------------------------------------------
    def _start_motion(
        self,
        motion_type: str,
        dist_m: float,
        yaw_rad: float,
        max_v: Optional[float] = None,
        max_w: Optional[float] = None,
    ) -> None:
        """Initializes trajectory profiler and captures baseline sensor positions."""
        self._publish_cmd_vel(0.0, 0.0)

        self.active_motion_type = motion_type
        self.target_dist_m = dist_m
        self.target_yaw_rad = yaw_rad
        self.motion_start_time = time.time()
        self.yaw_integral = 0.0
        self.turn_settle_start = None

        self.current_max_v = max_v if (max_v and max_v > 0) else self.max_linear_speed
        self.current_max_w = max_w if (max_w and max_w > 0) else self.max_angular_speed
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        # Baseline capture
        self.start_heading_rad = self.current_heading_rad if self.current_heading_rad is not None else 0.0
        self.start_unrolled_yaw = self.unrolled_yaw_rad
        self.start_odom_pos = self.current_odom_pos if (self.current_odom_pos and (time.time() - self.last_odom_time < 0.6)) else None

        # Reset GPS baseline and path odometer
        self.accumulated_gps_dist_m = 0.0
        if self.current_gps_coords and (time.time() - self.last_gps_time < 1.0):
            if not self.gps_require_rtk or self.gps_status in [NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.STATUS_SBAS_FIX]:
                self.start_gps_coords = self.current_gps_coords
                self.last_gps_accum_coords = self.current_gps_coords
            else:
                self.start_gps_coords = None
                self.last_gps_accum_coords = None
        else:
            self.start_gps_coords = None
            self.last_gps_accum_coords = None

        # Determine active distance tracking method for logging
        active_source = "TIME_FALLBACK"
        if (self.distance_source in ['auto', 'odom']) and self.start_odom_pos:
            active_source = "ODOM (LiDAR / rf2o)"
        elif (self.distance_source in ['auto', 'gps']) and self.start_gps_coords:
            active_source = "RTK_GPS (Waveshare LC29H)"

        # Stuck detection baseline
        self.last_moved_time = time.time()
        self.last_dist_m = 0.0
        self.last_yaw_rad = 0.0

        # Initialize Trajectory Profiler
        if motion_type == "STRAIGHT":
            self.profiler.start_straight(dist_m, max_v=max_v)
        elif motion_type == "ROTATE":
            self.profiler.start_rotate(yaw_rad, max_w=max_w)
        elif motion_type == "ARC":
            self.profiler.start_arc(dist_m, yaw_rad, max_v=max_v, max_w=max_w)

        self._set_state(ControllerState.EXECUTING)
        self.get_logger().info(
            f"[PRIMITIVE START] Type: {motion_type}, Dist: {dist_m:+.2f}m, Yaw: {math.degrees(yaw_rad):+.1f}°, Distance Source: [{active_source}]"
        )

    def _stop_motion(self) -> None:
        """Emergency or clean stop."""
        self._publish_cmd_vel(0.0, 0.0)
        self._set_state(ControllerState.IDLE)

    def _set_state(self, new_state: str) -> None:
        if self.state != new_state:
            self.state = new_state
            msg = String()
            msg.data = new_state
            self.status_pub.publish(msg)

    # --------------------------------------------------------------------------
    # Main 50 Hz Control Loop
    # --------------------------------------------------------------------------
    def _control_loop(self) -> None:
        if self.state != ControllerState.EXECUTING:
            return

        now = time.time()
        elapsed_t = now - self.motion_start_time

        # 1. Sample desired reference trajectory
        v_ref, omega_ref, s_ref, yaw_ref, profiler_finished = self.profiler.sample(elapsed_t)

        # 2. Measure actual state
        actual_dist = self._get_measured_distance_m()
        actual_yaw = self._get_measured_yaw_rad()

        # 3. Check for physical motion progress (Stuck / Stall Detector)
        dist_delta = abs(actual_dist - self.last_dist_m)
        yaw_delta = abs(actual_yaw - self.last_yaw_rad)

        has_physical_motion = (
            abs(self.current_yaw_rate) > 0.06  # BNO08x gyro is actively detecting rotation (> 3.5 deg/s)
            or dist_delta > 0.005              # Odometry distance moved > 5mm
            or yaw_delta > math.radians(0.3)   # Heading change > 0.3 deg
        )

        # Only command movement when active drive velocities were sent to motors
        is_commanding_movement = abs(self.last_cmd_v) > 0.05 or abs(self.last_cmd_w) > 0.15

        if has_physical_motion or not is_commanding_movement:
            # Robot is physically moving OR intentionally stopped (settling/standstill)
            self.last_moved_time = now
            self.last_dist_m = actual_dist
            self.last_yaw_rad = actual_yaw

        # Stall trigger: commanded to drive for > stuck_detect_time_s with ZERO physical motion
        # (Ignore stall watchdog when within 4.0 deg of target to prevent false aborts during final trim/settle)
        is_near_target = (
            (self.active_motion_type == "ROTATE" and abs(shortest_angular_diff_rad(self.target_yaw_rad, actual_yaw)) <= math.radians(4.0))
            or (self.active_motion_type in ["STRAIGHT", "ARC"] and (abs(self.target_dist_m) - abs(actual_dist)) <= 0.05)
        )

        if is_commanding_movement and not is_near_target and (now - self.last_moved_time > self.stuck_detect_time_s):
            self.get_logger().error(
                f"[STALL DETECTED] Zero motion detected for {self.stuck_detect_time_s:.2f}s while driving! Aborting for safety."
            )
            self._publish_cmd_vel(0.0, 0.0)
            self._set_state(ControllerState.STUCK)
            return

        # 4. Completion & Feedback Control
        dist_remaining = abs(self.target_dist_m) - abs(actual_dist)
        yaw_error_to_target = abs(shortest_angular_diff_rad(self.target_yaw_rad, actual_yaw))

        # --- SPECIALIZED LOGIC FOR IN-PLACE ROTATION ---
        if self.active_motion_type == "ROTATE":
            error_yaw = shortest_angular_diff_rad(self.target_yaw_rad, actual_yaw)
            abs_err = abs(error_yaw)

            # Standstill at Target Check:
            if abs_err <= self.yaw_tolerance_rad:
                self._publish_cmd_vel(0.0, 0.0)
                if self.turn_settle_start is None:
                    self.turn_settle_start = now

                is_stationary = abs(self.current_yaw_rate) < 0.08  # < 4.5 deg/s
                settle_elapsed = now - self.turn_settle_start

                if settle_elapsed >= self.turn_settle_time_s and is_stationary:
                    self._set_state(ControllerState.COMPLETED)
                    self.get_logger().info(
                        f"[ROTATE COMPLETED] Target: {math.degrees(self.target_yaw_rad):.1f}°, "
                        f"Settled: {math.degrees(actual_yaw):.1f}° (Final Error: {math.degrees(error_yaw):+.2f}°)"
                    )
                    return
                return

            # Outside tolerance: reset settle timer and drive towards target
            self.turn_settle_start = None
            sign = 1.0 if error_yaw >= 0 else -1.0

            # Smooth proportional deceleration into target:
            speed_factor = min(1.0, abs_err / math.radians(20.0))
            target_w = self.min_angular_speed + (self.current_max_w - self.min_angular_speed) * speed_factor
            cmd_omega = sign * target_w

            self._publish_cmd_vel(0.0, cmd_omega)
            return

        # --- LOGIC FOR STRAIGHT DRIVE & DRIVE ARC ---
        motion_complete = False
        if self.active_motion_type == "STRAIGHT":
            motion_complete = (dist_remaining <= self.distance_tolerance_m) and (profiler_finished or dist_remaining <= 0.005)
        elif self.active_motion_type == "ARC":
            # Check if this is a closed loop / full circle (target yaw ~ 360 deg = 2*pi)
            is_full_circle = abs(self.target_yaw_rad) >= (2.0 * math.pi - 0.25)

            # High-precision RTK loop closure: if full circle is > 80% complete and heading rotated > 315 deg
            if (
                is_full_circle
                and self.start_gps_coords
                and self.current_gps_coords
                and (time.time() - self.last_gps_time < 1.0)
                and actual_dist >= 0.80 * abs(self.target_dist_m)
                and abs(actual_yaw) >= (2.0 * math.pi - math.radians(45.0))
            ):
                loop_closure_err = gps_distance_m(
                    self.start_gps_coords[0], self.start_gps_coords[1],
                    self.current_gps_coords[0], self.current_gps_coords[1]
                )
                if loop_closure_err <= self.distance_tolerance_m:
                    self.get_logger().info(
                        f"[CIRCLE RTK LOOP CLOSURE] Target spot reached! Loop closure error: {loop_closure_err*100.0:.1f} cm"
                    )
                    motion_complete = True

            if not motion_complete:
                # Standard completion: distance reached AND heading reached, or profiler finished with remaining dist satisfied
                dist_done = dist_remaining <= self.distance_tolerance_m
                yaw_done = abs(yaw_error_to_target) <= self.yaw_tolerance_rad
                motion_complete = (dist_done and yaw_done) or (dist_done and profiler_finished) or (dist_remaining <= 0.005)

        if motion_complete:
            self._publish_cmd_vel(0.0, 0.0)
            self._set_state(ControllerState.COMPLETED)
            self.get_logger().info(
                f"[PRIMITIVE COMPLETED] Final Dist: {actual_dist:.2f}m, Final Yaw: {math.degrees(actual_yaw):.1f}°"
            )
            return

        # Dual-Rate Feedback Controller for Straight & Arc
        yaw_error = shortest_angular_diff_rad(yaw_ref, actual_yaw)
        dt = 1.0 / self.control_rate_hz
        self.yaw_integral = max(-1.0, min(1.0, self.yaw_integral + yaw_error * dt))

        p_yaw = self.yaw_kp * yaw_error
        i_yaw = self.yaw_ki * self.yaw_integral
        # Clamp steering trim authority to +/- 0.40 rad/s to prevent track chatter
        yaw_trim = max(-0.40, min(0.40, p_yaw + i_yaw))

        # Closed-loop distance braking: prevent forward overshoot if motor speed runs ahead of profile
        cmd_v = v_ref
        if abs(self.target_dist_m) > 0.05 and dist_remaining < 0.25:
            # v_limit = sqrt(2 * a * dist_remaining)
            decel_v_limit = math.sqrt(max(0.0, 2.0 * self.max_linear_accel * max(0.0, dist_remaining)))
            if abs(cmd_v) > decel_v_limit:
                cmd_v = math.copysign(max(self.min_linear_speed, decel_v_limit), cmd_v) if dist_remaining > self.distance_tolerance_m else 0.0

        if self.active_motion_type == "ARC":
            cmd_omega = self.profiler.curvature * cmd_v + yaw_trim
        else:
            cmd_omega = omega_ref + yaw_trim

        # Enforce minimum linear speed only when active distance remains to drive
        if abs(cmd_v) > 1e-4 and abs(cmd_v) < self.min_linear_speed and dist_remaining > self.distance_tolerance_m:
            cmd_v = math.copysign(self.min_linear_speed, cmd_v)

        self._publish_cmd_vel(cmd_v, cmd_omega)

    # --------------------------------------------------------------------------
    # Sensor State Helpers
    # --------------------------------------------------------------------------
    def _get_measured_yaw_rad(self) -> float:
        """Returns continuous unrolled heading change in radians since motion start."""
        if self.current_heading_rad is None:
            return 0.0
        return self.unrolled_yaw_rad - self.start_unrolled_yaw

    def _get_measured_distance_m(self) -> float:
        """Returns distance traveled in meters from active sensor source."""
        # 1. 2D Laser Odometry (rf2o) - LiDAR Robot
        if (
            (self.distance_source in ['auto', 'odom'])
            and self.start_odom_pos
            and self.current_odom_pos
            and (time.time() - self.last_odom_time < 0.6)
        ):
            dx = self.current_odom_pos[0] - self.start_odom_pos[0]
            dy = self.current_odom_pos[1] - self.start_odom_pos[1]
            return math.sqrt(dx * dx + dy * dy)

        # 2. RTK GNSS (Waveshare LC29H) - GPS Robot
        if (
            (self.distance_source in ['auto', 'gps'])
            and self.start_gps_coords
            and self.current_gps_coords
            and (time.time() - self.last_gps_time < 1.0)
        ):
            if self.active_motion_type == "ARC":
                # For curved paths & circles, return accumulated path distance along trajectory
                return self.accumulated_gps_dist_m
            else:
                # For straight lines, return direct Euclidean distance from start
                return gps_distance_m(
                    self.start_gps_coords[0], self.start_gps_coords[1],
                    self.current_gps_coords[0], self.current_gps_coords[1]
                )

        # 3. Universal Time-integration fallback
        if self.motion_start_time is not None:
            elapsed = time.time() - self.motion_start_time
            return elapsed * self.default_linear_speed

        return 0.0

    def _publish_cmd_vel(self, vx: float, wz: float) -> None:
        """Sends velocity target to the motor controller node."""
        self.last_cmd_v = float(vx)
        self.last_cmd_w = float(wz)
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

    def _publish_diagnostics(self) -> None:
        """Publishes lightweight Pi 5 SoC diagnostics."""
        try:
            temp_c = None
            if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    val = f.read().strip()
                    if val:
                        temp_c = round(float(val) / 1000.0, 1)

            diag = {
                "cpu_temp": f"{temp_c:.1f} °C" if temp_c else "N/A",
                "battery_v": f"{self.battery_voltage:.2f} V",
                "state": self.state,
                "motion_type": self.active_motion_type,
            }
            msg = String()
            msg.data = json.dumps(diag)
            self.diag_pub.publish(msg)

            # Periodic status heartbeat
            status_msg = String()
            status_msg.data = self.state
            self.status_pub.publish(status_msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._publish_cmd_vel(0.0, 0.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PrimitiveMotionController()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

