#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: Unified State-Driven Motion Primitive Controller
==============================================================================
Target Platform: Raspberry Pi 5 / ROS 2 Jazzy
Hardware: Cytron MDD10, MikroE BNO08x IMU, Waveshare LC29H RTK GPS / YDLIDAR T-mini Plus

Description:
    Executes high-level motion primitives (STRAIGHT, ROTATE, ARC, STOP) purely as
    a function of measured spatial state (distance X and angle Y) — NEVER open-loop
    wall-clock time.

    Unified Sensor Abstraction ("gps" vs "odom"):
      - Both RTK GPS (/fix) and LiDAR Odometry (/odom) feed the exact same 2D
        planar displacement tracker (chord_dist_m and accumulated_path_dist_m).
      - Switching between the GPS robot and the LiDAR robot requires changing ONLY
        a single parameter (`distance_source: "gps"` vs `"odom"`).

Publishes:
    - `/cmd_vel` (geometry_msgs/msg/Twist): Velocity commands to mdd10_motor_controller.
    - `/motion_status` (std_msgs/msg/String): State machine status (IDLE, EXECUTING, etc.).
    - `/step_status` (std_msgs/msg/String): Detailed progress for Fleet Dashboard.
    - `/robot/diagnostics` (std_msgs/msg/String): SoC temperature & battery voltage.
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
    return (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi


def gps_enu_delta_m(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Compute flat-Earth East-North metric offset (dE, dN) in meters from (lat1, lon1) to (lat2, lon2).
    Accurate to millimeter level for local ranges (< 1 km).
    """
    earth_radius_m = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) * 0.5)
    d_east = d_lon * math.cos(lat_avg) * earth_radius_m
    d_north = d_lat * earth_radius_m
    return d_east, d_north


def gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute flat-Earth Euclidean distance in meters between two WGS84 coordinates."""
    de, dn = gps_enu_delta_m(lat1, lon1, lat2, lon2)
    return math.hypot(de, dn)


class ControllerState:
    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    CANCELING = "CANCELING"
    COMPLETED = "COMPLETED"
    STUCK = "STUCK"
    SENSOR_ERROR = "SENSOR_ERROR"


class PrimitiveMotionController(Node):
    """
    Executes high-level motion primitives (STRAIGHT, ROTATE, ARC) via pure state-based
    feedback (BNO08x IMU + pluggable GPS / LiDAR Odometry distance source).
    """

    def __init__(self) -> None:
        super().__init__('primitive_motion_controller')
        self.cb_group = ReentrantCallbackGroup()

        # ----------------------------------------------------------------------
        # ROS 2 Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('control_rate_hz', 50.0)         # Inner loop rate (matches 50 Hz IMU)
        self.declare_parameter('feedback_rate_hz', 10.0)        # Action feedback publication rate

        # Speed & Acceleration Limits
        self.declare_parameter('default_linear_speed', 0.20)    # Cruising linear speed (m/s)
        self.declare_parameter('default_angular_speed', 1.00)   # Cruising angular speed (rad/s)
        self.declare_parameter('max_linear_speed', 0.50)        # Hard limit on linear speed (m/s)
        self.declare_parameter('max_angular_speed', 1.80)       # Hard limit on angular speed (rad/s)
        self.declare_parameter('max_linear_accel', 0.35)        # Linear acceleration / deceleration (m/s^2)
        self.declare_parameter('max_angular_accel', 1.50)       # Angular acceleration / deceleration (rad/s^2)

        # Minimum Speeds (to overcome static track friction)
        self.declare_parameter('min_linear_speed', 0.08)        # Minimum linear speed before target stop (m/s)
        self.declare_parameter('min_angular_speed', 0.65)       # Minimum pivot speed to overcome track scrubbing (rad/s)

        # Closed-Loop Feedback Gains (Standard ROS CCW+ convention)
        self.declare_parameter('yaw_kp', 2.0)                   # Proportional gain for heading lock (rad/s per rad)
        self.declare_parameter('yaw_ki', 0.60)                  # Integral gain to trim left/right motor asymmetry
        self.declare_parameter('yaw_kd', 0.20)                  # Gyro rate damping gain
        self.declare_parameter('yaw_tolerance_rad', math.radians(1.8))  # Target yaw tolerance (~1.8 deg)
        self.declare_parameter('turn_settle_time_s', 0.15)      # Standstill verification window at target (s)
        self.declare_parameter('distance_tolerance_m', 0.03)    # Target distance tolerance (3 cm)

        # Stuck Watchdog & Pluggable Distance Sensor Configuration
        # SINGLE SWITCH FOR ROBOT TYPE: 'gps' (GPS Robot) or 'odom' (LiDAR Robot) or 'auto'
        self.declare_parameter('distance_source', 'gps')
        self.declare_parameter('gps_require_rtk', False)        # False = allow 3D/DGPS/RTK; True = require RTK Fix/Float
        self.declare_parameter('stuck_detect_time_s', 2.5)      # Stall abort timeout when zero motion occurs (s)

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
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_ki = float(self.get_parameter('yaw_ki').value)
        self.yaw_kd = float(self.get_parameter('yaw_kd').value)
        self.yaw_tolerance_rad = float(self.get_parameter('yaw_tolerance_rad').value)
        self.turn_settle_time_s = float(self.get_parameter('turn_settle_time_s').value)
        self.distance_tolerance_m = float(self.get_parameter('distance_tolerance_m').value)
        self.stuck_detect_time_s = float(self.get_parameter('stuck_detect_time_s').value)
        self.distance_source = str(self.get_parameter('distance_source').value).lower()
        self.gps_require_rtk = bool(self.get_parameter('gps_require_rtk').value)

        # ----------------------------------------------------------------------
        # State-Based Motion Profiler
        # ----------------------------------------------------------------------
        self.profiler = MotionPrimitiveProfiler(
            default_linear_speed=self.default_linear_speed,
            default_angular_speed=self.default_angular_speed,
            max_linear_accel=self.max_linear_accel,
            max_angular_accel=self.max_angular_accel,
            min_linear_speed=self.min_linear_speed,
            min_angular_speed=self.min_angular_speed,
        )

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/motion_status', 10)
        self.step_status_pub = self.create_publisher(String, '/step_status', 10)
        self.diag_pub = self.create_publisher(String, '/robot/diagnostics', 10)

        # Topic Command Subscriptions
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
                    goal_callback=self._goal_callback,
                    cancel_callback=self._cancel_action_callback,
                    callback_group=self.cb_group,
                )
                self.get_logger().info("Action Server [execute_motion] initialized successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Action Server: {e}")
        else:
            self.get_logger().warn(
                f"cone_robot_interfaces action not available ({_ACTION_IMPORT_ERROR})! "
                "Running in topic mode (/cmd_step, /cmd_primitive)."
            )

        # ----------------------------------------------------------------------
        # Internal State Variables
        # ----------------------------------------------------------------------
        self.state = ControllerState.IDLE

        # IMU State (CCW+ radians, continuous unrolled yaw)
        self.current_heading_rad: Optional[float] = None
        self.unrolled_yaw_rad: float = 0.0
        self.start_unrolled_yaw: float = 0.0
        self.current_yaw_rate: float = 0.0
        self.last_heading_time: float = 0.0

        # Unified 2D Position & Distance State (Populated by either GPS or LiDAR Odom)
        self.active_sensor_mode: str = "NONE"   # "GPS" or "ODOM"
        self.chord_dist_m: float = 0.0          # Direct Euclidean distance from start (m)
        self.accumulated_path_dist_m: float = 0.0  # Incremental path length along curve/line (m)

        # Raw Odom Sensor State
        self.current_odom_pos: Optional[Tuple[float, float]] = None
        self.start_odom_pos: Optional[Tuple[float, float]] = None
        self.last_odom_accum_pos: Optional[Tuple[float, float]] = None
        self.last_odom_time: float = 0.0

        # Raw GPS Sensor State
        self.current_gps_coords: Optional[Tuple[float, float]] = None
        self.start_gps_coords: Optional[Tuple[float, float]] = None
        self.last_gps_accum_coords: Optional[Tuple[float, float]] = None
        self.gps_status: int = NavSatStatus.STATUS_NO_FIX
        self.last_gps_time: float = 0.0

        # Battery State
        self.battery_voltage: float = 12.0

        # Active Motion Goal
        self.active_motion_type = "NONE"
        self.target_dist_m: float = 0.0
        self.target_yaw_rad: float = 0.0
        self.yaw_integral: float = 0.0
        self.turn_settle_start: Optional[float] = None

        # Stuck Watchdog & Command Tracking
        self.last_moved_time: float = 0.0
        self.last_dist_m: float = 0.0
        self.last_yaw_rad: float = 0.0
        self.last_cmd_v: float = 0.0
        self.last_cmd_w: float = 0.0
        self.current_max_v: float = self.max_linear_speed
        self.current_max_w: float = self.max_angular_speed
        self._last_status_pub_time: float = 0.0

        # Action Execution Reference
        self._current_goal_handle = None
        self._cancel_requested: bool = False
        self._preempted_by_topic: bool = False

        # Timers
        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.diag_timer = self.create_timer(2.0, self._publish_diagnostics)

        self.get_logger().info("==================================================")
        self.get_logger().info(" Unified State-Driven Motion Controller Ready")
        self.get_logger().info(f" Control Rate   : {self.control_rate_hz:.0f} Hz (Zero Time-Fallback)")
        self.get_logger().info(f" Distance Source: {self.distance_source.upper()}")
        self.get_logger().info("==================================================")

    # --------------------------------------------------------------------------
    # Sensor Callbacks
    # --------------------------------------------------------------------------
    def _heading_callback(self, msg: Float32) -> None:
        """Converts [0, 360) deg CCW heading into [-pi, pi] rad and unrolls continuous yaw."""
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
        self.current_yaw_rate = float(msg.angular_velocity.z)

    def _odom_callback(self, msg: Odometry) -> None:
        curr_x = float(msg.pose.pose.position.x)
        curr_y = float(msg.pose.pose.position.y)
        self.current_odom_pos = (curr_x, curr_y)
        self.last_odom_time = time.time()

        if self.state == ControllerState.EXECUTING and self.active_sensor_mode == "ODOM":
            if self.start_odom_pos is not None:
                self.chord_dist_m = math.hypot(
                    curr_x - self.start_odom_pos[0],
                    curr_y - self.start_odom_pos[1],
                )
            if self.last_odom_accum_pos is not None:
                step_m = math.hypot(
                    curr_x - self.last_odom_accum_pos[0],
                    curr_y - self.last_odom_accum_pos[1],
                )
                if step_m > 0.50:
                    self.last_odom_accum_pos = (curr_x, curr_y)
                elif step_m >= 0.004 and abs(self.last_cmd_v) > 0.01:
                    self.accumulated_path_dist_m += step_m
                    self.last_odom_accum_pos = (curr_x, curr_y)

    def _fix_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        curr_lat = float(msg.latitude)
        curr_lon = float(msg.longitude)
        self.current_gps_coords = (curr_lat, curr_lon)
        self.gps_status = int(msg.status.status)
        self.last_gps_time = time.time()

        if self.state == ControllerState.EXECUTING and self.active_sensor_mode == "GPS":
            if not self._is_gps_fix_acceptable():
                return
            if self.start_gps_coords is not None:
                self.chord_dist_m = gps_distance_m(
                    self.start_gps_coords[0], self.start_gps_coords[1],
                    curr_lat, curr_lon,
                )
            if self.last_gps_accum_coords is not None:
                step_m = gps_distance_m(
                    self.last_gps_accum_coords[0], self.last_gps_accum_coords[1],
                    curr_lat, curr_lon,
                )
                # Filter out static noise (< 1.2 cm) and multipath jumps (> 60 cm)
                if step_m > 0.60:
                    self.last_gps_accum_coords = (curr_lat, curr_lon)
                elif step_m >= 0.012 and abs(self.last_cmd_v) > 0.01:
                    self.accumulated_path_dist_m += step_m
                    self.last_gps_accum_coords = (curr_lat, curr_lon)

    def _battery_callback(self, msg: BatteryState) -> None:
        if msg.voltage > 5.0:
            self.battery_voltage = float(msg.voltage)

    def _is_gps_fix_acceptable(self) -> bool:
        """Checks whether current GPS fix status meets configuration requirements."""
        if self.gps_require_rtk:
            return self.gps_status in [NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.STATUS_SBAS_FIX]
        return self.gps_status >= NavSatStatus.STATUS_FIX

    # --------------------------------------------------------------------------
    # Topic Command Callbacks (/cmd_step and /cmd_primitive)
    # --------------------------------------------------------------------------
    def _cmd_step_callback(self, msg: Vector3) -> None:
        """
        Receives /cmd_step:
          msg.x = distance in cm (+ forward, - reverse)
          msg.z = turn angle in degrees (+ CCW left, - CW right)
        """
        dist_m = float(msg.x) / 100.0
        yaw_rad = math.radians(float(msg.z))
        self._dispatch_topic_command(dist_m, yaw_rad)

    def _cmd_primitive_callback(self, msg: Vector3) -> None:
        """
        Receives /cmd_primitive:
          msg.x = distance in meters (+ forward, - reverse)
          msg.z = delta yaw in radians (+ CCW left, - CW right)
        """
        dist_m = float(msg.x)
        yaw_rad = float(msg.z)
        self._dispatch_topic_command(dist_m, yaw_rad)

    def _dispatch_topic_command(self, dist_m: float, yaw_rad: float) -> None:
        if self._current_goal_handle is not None:
            self.get_logger().warn(
                "[PREEMPT] Active Action goal preempted by incoming topic command."
            )
            self._preempted_by_topic = True

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
    def _goal_callback(self, goal_request):
        # Always allow TYPE_STOP (3) to execute immediately
        if getattr(goal_request, 'motion_type', None) == 3:
            self._cancel_requested = True
            return GoalResponse.ACCEPT

        if self.state == ControllerState.EXECUTING or self._current_goal_handle is not None:
            self.get_logger().warn(
                "[ACTION] Rejecting new motion goal: Controller is currently busy."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_action_callback(self, goal_handle):
        self.get_logger().info("[ACTION] Preemption / Cancel requested by client.")
        self._cancel_requested = True
        self._stop_motion()
        return CancelResponse.ACCEPT

    def _execute_action_callback(self, goal_handle):
        self._current_goal_handle = goal_handle
        self._cancel_requested = False
        self._preempted_by_topic = False
        goal = goal_handle.request

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

        # Start State-Driven Motion
        started_ok, err_msg = self._start_motion(
            m_type,
            float(goal.distance),
            float(goal.delta_yaw),
            max_v=float(goal.max_velocity) if goal.max_velocity > 0 else None,
            max_w=float(goal.max_yaw_rate) if goal.max_yaw_rate > 0 else None,
        )

        if not started_ok:
            goal_handle.abort()
            result = ExecuteMotion.Result()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_INVALID_GOAL
            result.message = err_msg
            self._current_goal_handle = None
            self._set_state(ControllerState.IDLE, f"Aborted: {err_msg}")
            return result

        feedback = ExecuteMotion.Feedback()
        rate = self.create_rate(self.feedback_rate_hz)

        # Wait loop until state-driven motion finishes, aborts, or cancels
        while rclpy.ok() and self.state == ControllerState.EXECUTING:
            if self._preempted_by_topic:
                goal_handle.abort()
                result = ExecuteMotion.Result()
                result.success = False
                result.error_code = ExecuteMotion.Result.ERROR_CANCELED
                result.message = "Action preempted by incoming topic command"
                result.actual_distance = float(self._get_measured_distance_m())
                result.actual_yaw = float(self._get_measured_yaw_rad())
                self._current_goal_handle = None
                self._preempted_by_topic = False
                return result

            if self._cancel_requested or goal_handle.is_cancel_requested:
                self._stop_motion()
                self._set_state(ControllerState.CANCELING, "Canceled")
                goal_handle.canceled()
                result = ExecuteMotion.Result()
                result.success = False
                result.error_code = ExecuteMotion.Result.ERROR_CANCELED
                result.message = "Motion canceled by client"
                result.actual_distance = float(self._get_measured_distance_m())
                result.actual_yaw = float(self._get_measured_yaw_rad())
                self._current_goal_handle = None
                self._set_state(ControllerState.IDLE, "Idle")
                return result

            measured_dist = self._get_measured_distance_m()
            measured_yaw = self._get_measured_yaw_rad()

            if self.active_motion_type == "ROTATE":
                ratio = abs(measured_yaw) / max(1e-4, abs(self.target_yaw_rad))
            else:
                ratio = abs(measured_dist) / max(1e-4, abs(self.target_dist_m))

            feedback.progress_ratio = float(min(1.0, max(0.0, ratio)))
            feedback.distance_remaining = float(max(0.0, abs(self.target_dist_m) - abs(measured_dist)))
            feedback.yaw_remaining = float(max(0.0, abs(self.target_yaw_rad) - abs(measured_yaw)))
            feedback.current_velocity = float(self.last_cmd_v)
            feedback.current_yaw_rate = float(self.current_yaw_rate)
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Evaluate final outcome
        result = ExecuteMotion.Result()
        result.actual_distance = float(self._get_measured_distance_m())
        result.actual_yaw = float(self._get_measured_yaw_rad())

        if self._preempted_by_topic:
            goal_handle.abort()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_CANCELED
            result.message = "Action preempted by incoming topic command"
            self._current_goal_handle = None
            self._preempted_by_topic = False
            return result

        if self.state == ControllerState.STUCK:
            goal_handle.abort()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_STUCK
            result.message = "Robot stall / stuck detected by watchdog"
        else:
            goal_handle.succeed()
            result.success = True
            result.error_code = ExecuteMotion.Result.ERROR_NONE
            result.message = (
                f"Motion complete (dist={result.actual_distance:.2f}m, "
                f"yaw={math.degrees(result.actual_yaw):+.1f} deg)"
            )

        self._current_goal_handle = None
        self._set_state(ControllerState.IDLE, "Idle")
        return result

    # --------------------------------------------------------------------------
    # Motion Initialization & Pluggable Sensor Binding
    # --------------------------------------------------------------------------
    def _resolve_active_sensor(self, needs_distance: bool) -> Tuple[bool, str]:
        """
        Verifies required sensors are online and binds either 'GPS' or 'ODOM'.
        Never falls back to open-loop time integration.
        """
        now = time.time()

        # 1. IMU is always required for closed-loop heading lock & rotation
        if self.current_heading_rad is None or (now - self.last_heading_time > 1.5):
            return False, "IMU (/imu/heading) is offline or not publishing!"

        if not needs_distance:
            self.active_sensor_mode = "IMU_ONLY"
            return True, "IMU (50 Hz)"

        odom_fresh = (self.current_odom_pos is not None) and (now - self.last_odom_time < 1.5)
        gps_fresh = (
            (self.current_gps_coords is not None)
            and (now - self.last_gps_time < 2.0)
            and self._is_gps_fix_acceptable()
        )

        # Explicit GPS mode (GPS Robot)
        if self.distance_source == 'gps':
            if not gps_fresh:
                req_str = "RTK Fix/Float" if self.gps_require_rtk else "valid 3D/RTK Fix"
                return False, f"GPS (/fix) not ready or missing {req_str} (status={self.gps_status})!"
            self.active_sensor_mode = "GPS"
            return True, "GPS (/fix)"

        # Explicit ODOM mode (LiDAR Robot)
        if self.distance_source == 'odom':
            if not odom_fresh:
                return False, "LiDAR Odometry (/odom) is not publishing!"
            self.active_sensor_mode = "ODOM"
            return True, "ODOM (/odom)"

        # Auto mode: prefer GPS if fresh, else ODOM
        if gps_fresh:
            self.active_sensor_mode = "GPS"
            return True, "GPS (/fix)"
        if odom_fresh:
            self.active_sensor_mode = "ODOM"
            return True, "ODOM (/odom)"

        return False, "Neither GPS (/fix) nor LiDAR Odometry (/odom) is available! Refusing time-based drive."

    def _start_motion(
        self,
        motion_type: str,
        dist_m: float,
        yaw_rad: float,
        max_v: Optional[float] = None,
        max_w: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """Initializes state-driven trajectory profiler and locks sensor baselines."""
        self._publish_cmd_vel(0.0, 0.0)

        needs_distance = motion_type in ["STRAIGHT", "ARC"]
        sensor_ok, sensor_desc = self._resolve_active_sensor(needs_distance)
        if not sensor_ok:
            self.get_logger().error(f"[START REJECTED] {sensor_desc}")
            self._set_state(ControllerState.SENSOR_ERROR, sensor_desc)
            return False, sensor_desc

        self.active_motion_type = motion_type
        self.target_dist_m = float(dist_m)
        self.target_yaw_rad = float(yaw_rad)
        self.yaw_integral = 0.0
        self.turn_settle_start = None

        self.current_max_v = max_v if (max_v and max_v > 0.0) else self.default_linear_speed
        self.current_max_w = max_w if (max_w and max_w > 0.0) else self.default_angular_speed
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        # Lock IMU Heading Baseline
        self.start_unrolled_yaw = self.unrolled_yaw_rad

        # Lock 2D Position & Path Distance Baselines
        self.chord_dist_m = 0.0
        self.accumulated_path_dist_m = 0.0
        self.start_odom_pos = self.current_odom_pos
        self.last_odom_accum_pos = self.current_odom_pos
        self.start_gps_coords = self.current_gps_coords
        self.last_gps_accum_coords = self.current_gps_coords

        # Stuck Watchdog Baseline
        self.last_moved_time = time.time()
        self.last_dist_m = 0.0
        self.last_yaw_rad = 0.0

        # Configure State-Driven Profiler
        if motion_type == "STRAIGHT":
            self.profiler.start_straight(dist_m, max_v=self.current_max_v)
        elif motion_type == "ROTATE":
            self.profiler.start_rotate(yaw_rad, max_w=self.current_max_w)
        elif motion_type == "ARC":
            self.profiler.start_arc(dist_m, yaw_rad, max_v=self.current_max_v, max_w=self.current_max_w)

        self._set_state(
            ControllerState.EXECUTING,
            f"{motion_type}: 0.00/{abs(dist_m):.2f}m ({math.degrees(yaw_rad):+.0f}°)",
        )
        self.get_logger().info(
            f"[PRIMITIVE START] Type: {motion_type} | Target Dist: {dist_m:+.2f}m | "
            f"Target Yaw: {math.degrees(yaw_rad):+.1f}° | Sensor: [{sensor_desc}]"
        )
        return True, "OK"

    def _stop_motion(self) -> None:
        """Immediate clean stop."""
        self._publish_cmd_vel(0.0, 0.0)
        self._set_state(ControllerState.IDLE, "Idle")

    def _set_state(self, new_state: str, detail_str: Optional[str] = None) -> None:
        if self.state != new_state:
            self.state = new_state
            msg = String()
            msg.data = new_state
            self.status_pub.publish(msg)

        if detail_str:
            step_msg = String()
            step_msg.data = detail_str
            self.step_status_pub.publish(step_msg)

    # --------------------------------------------------------------------------
    # Main 50 Hz State-Driven Control Loop (Zero Time-Based Trajectory Sampling)
    # --------------------------------------------------------------------------
    def _control_loop(self) -> None:
        if self.state != ControllerState.EXECUTING:
            return

        now = time.time()
        dt = 1.0 / self.control_rate_hz

        # 1. Measure actual physical distance and heading change
        actual_dist = self._get_measured_distance_m()
        actual_yaw = self._get_measured_yaw_rad()

        # 2. Sample desired velocity & heading reference from MEASURED STATE (not time!)
        v_ref, omega_ref, _s_ref, yaw_ref, dist_or_rot_done = self.profiler.sample_by_state(
            actual_dist_m=actual_dist,
            actual_yaw_rad=actual_yaw,
            dt=dt,
            dist_tolerance_m=self.distance_tolerance_m,
            yaw_tolerance_rad=self.yaw_tolerance_rad,
        )

        # 3. Physical Motion Progress Check (Stuck / Stall Watchdog)
        dist_delta = abs(actual_dist - self.last_dist_m)
        yaw_delta = abs(actual_yaw - self.last_yaw_rad)

        has_physical_motion = (
            abs(self.current_yaw_rate) > 0.05
            or dist_delta > 0.008
            or yaw_delta > math.radians(0.4)
        )
        is_commanding_movement = abs(self.last_cmd_v) > 0.04 or abs(self.last_cmd_w) > 0.15

        if has_physical_motion or not is_commanding_movement:
            self.last_moved_time = now
            if dist_delta > 0.008:
                self.last_dist_m = actual_dist
            if yaw_delta > math.radians(0.4):
                self.last_yaw_rad = actual_yaw

        dist_remaining = max(0.0, abs(self.target_dist_m) - abs(actual_dist))
        yaw_error_to_target = self.target_yaw_rad - actual_yaw

        is_near_target = (
            (self.active_motion_type == "ROTATE" and abs(yaw_error_to_target) <= math.radians(4.0))
            or (self.active_motion_type in ["STRAIGHT", "ARC"] and dist_remaining <= 0.06)
        )

        if is_commanding_movement and not is_near_target and (now - self.last_moved_time > self.stuck_detect_time_s):
            self.get_logger().error(
                f"[STALL DETECTED] No physical progress for {self.stuck_detect_time_s:.1f}s! Aborting for safety."
            )
            self._publish_cmd_vel(0.0, 0.0)
            self._set_state(ControllerState.STUCK, "STUCK / STALLED")
            return

        # Periodic human-readable progress update for Fleet Dashboard (5 Hz)
        if now - self._last_status_pub_time >= 0.20:
            self._last_status_pub_time = now
            if self.active_motion_type == "ROTATE":
                prog_txt = f"Turning: {math.degrees(actual_yaw):+.1f}° / {math.degrees(self.target_yaw_rad):+.1f}°"
            elif self.active_motion_type == "ARC":
                prog_txt = (
                    f"Driving Arc: {actual_dist:.2f}/{abs(self.target_dist_m):.2f}m "
                    f"({math.degrees(actual_yaw):+.0f}°/{math.degrees(self.target_yaw_rad):+.0f}°)"
                )
            else:
                prog_txt = (
                    f"Driving Straight: {actual_dist:.2f}/{abs(self.target_dist_m):.2f}m "
                    f"(Yaw: {math.degrees(actual_yaw):+.1f}°)"
                )
            step_msg = String()
            step_msg.data = prog_txt
            self.step_status_pub.publish(step_msg)

        # ----------------------------------------------------------------------
        # MODE A: ROTATE IN PLACE (Pure IMU Closed-Loop Angle Control)
        # ----------------------------------------------------------------------
        if self.active_motion_type == "ROTATE":
            # Deadband hysteresis: once settling has started within yaw_tolerance_rad,
            # stay in settle mode unless overshoot exceeds 2x yaw_tolerance_rad.
            in_settle_hysteresis = (
                self.turn_settle_start is not None
                and abs(yaw_error_to_target) <= (self.yaw_tolerance_rad * 2.0)
            )
            if dist_or_rot_done or in_settle_hysteresis:
                self._publish_cmd_vel(0.0, 0.0)
                if self.turn_settle_start is None:
                    self.turn_settle_start = now
                is_stationary = abs(self.current_yaw_rate) < 0.08
                if (now - self.turn_settle_start) >= self.turn_settle_time_s and is_stationary:
                    self._set_state(ControllerState.COMPLETED, "Completed")
                    self.get_logger().info(
                        f"[ROTATE COMPLETED] Target: {math.degrees(self.target_yaw_rad):+.1f}°, "
                        f"Actual: {math.degrees(actual_yaw):+.1f}° (Err: {math.degrees(yaw_error_to_target):+.2f}°)"
                    )
                return

            self.turn_settle_start = None
            self._publish_cmd_vel(0.0, omega_ref)
            return

        # ----------------------------------------------------------------------
        # MODE B: STRAIGHT DRIVE & ARC DRIVE (Position + IMU Heading Closed-Loop)
        # ----------------------------------------------------------------------
        # Check full circle loop-closure shortcut for 360-deg arcs
        is_full_circle = (
            self.active_motion_type == "ARC"
            and abs(self.target_yaw_rad) >= (2.0 * math.pi - 0.25)
        )
        if (
            is_full_circle
            and actual_dist >= 0.80 * abs(self.target_dist_m)
            and abs(actual_yaw) >= (2.0 * math.pi - math.radians(40.0))
            and self.chord_dist_m <= self.distance_tolerance_m * 1.5
        ):
            self.get_logger().info(
                f"[ARC LOOP CLOSURE] Start position reached (err={self.chord_dist_m*100.0:.1f} cm)."
            )
            dist_or_rot_done = True

        # When linear distance X is reached:
        if dist_or_rot_done:
            # For ARC: ensure final target heading Y is also satisfied within tolerance
            if self.active_motion_type == "ARC" and abs(yaw_error_to_target) > self.yaw_tolerance_rad:
                turn_sign = 1.0 if yaw_error_to_target >= 0.0 else -1.0
                speed_factor = min(1.0, abs(yaw_error_to_target) / math.radians(20.0))
                trim_w = self.min_angular_speed + (self.current_max_w - self.min_angular_speed) * speed_factor
                self._publish_cmd_vel(0.0, turn_sign * trim_w)
                return

            self._publish_cmd_vel(0.0, 0.0)
            self._set_state(ControllerState.COMPLETED, "Completed")
            self.get_logger().info(
                f"[{self.active_motion_type} COMPLETED] Dist: {actual_dist:.3f}m "
                f"(Target: {abs(self.target_dist_m):.2f}m) | "
                f"Yaw: {math.degrees(actual_yaw):+.1f}° (Target: {math.degrees(self.target_yaw_rad):+.1f}°)"
            )
            return

        # 50 Hz Closed-Loop IMU Heading & Curvature Controller
        # For STRAIGHT: yaw_ref = 0.0 (locks initial heading).
        # For ARC:      yaw_ref = (s / |X|) * Y (tracks exact geometric arc curvature).
        yaw_error = yaw_ref - actual_yaw
        self.yaw_integral = max(-0.8, min(0.8, self.yaw_integral + yaw_error * dt))

        p_yaw = self.yaw_kp * yaw_error
        i_yaw = self.yaw_ki * self.yaw_integral
        d_yaw = self.yaw_kd * (omega_ref - self.current_yaw_rate)
        yaw_trim = max(-1.0, min(1.0, p_yaw + i_yaw + d_yaw))

        cmd_v = v_ref
        cmd_omega = omega_ref + yaw_trim

        self._publish_cmd_vel(cmd_v, cmd_omega)

    # --------------------------------------------------------------------------
    # Unified Sensor State Helpers (Identical for GPS and LiDAR Odom)
    # --------------------------------------------------------------------------
    def _get_measured_yaw_rad(self) -> float:
        """Returns continuous unrolled heading change in radians (+CCW / -CW) since motion start."""
        if self.current_heading_rad is None:
            return 0.0
        return self.unrolled_yaw_rad - self.start_unrolled_yaw

    def _get_measured_distance_m(self) -> float:
        """
        Returns physical ground distance traveled in meters from the active position sensor
        (GPS or LiDAR Odom). Never uses elapsed time.
        """
        if self.active_motion_type == "ROTATE":
            return 0.0

        if self.active_motion_type == "STRAIGHT":
            return self.chord_dist_m

        if self.active_motion_type == "ARC":
            # For arcs <= 125 deg, convert direct chord displacement into exact arc length
            # using measured heading change: s = chord * (|yaw|/2) / sin(|yaw|/2).
            # This is immune to incremental jitter while accurately measuring curved path length.
            abs_yaw = abs(self._get_measured_yaw_rad())
            if abs(self.target_yaw_rad) <= math.radians(125.0) and abs_yaw <= math.radians(135.0):
                if abs_yaw > 0.05:
                    half_yaw = 0.5 * abs_yaw
                    arc_factor = half_yaw / max(1e-4, math.sin(half_yaw))
                    return max(self.chord_dist_m * arc_factor, self.accumulated_path_dist_m)
                return self.chord_dist_m
            # For large arcs (> 125 deg / U-turns / full circles), use incremental path odometer
            return max(self.accumulated_path_dist_m, self.chord_dist_m)

        return self.chord_dist_m

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
                "sensor_mode": self.active_sensor_mode,
            }
            msg = String()
            msg.data = json.dumps(diag)
            self.diag_pub.publish(msg)

            status_msg = String()
            status_msg.data = self.state
            self.status_pub.publish(status_msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        try:
            if rclpy.ok():
                self._publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
