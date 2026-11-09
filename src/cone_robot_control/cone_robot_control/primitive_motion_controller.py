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


def gps_enu_to_latlon(ref_lat: float, ref_lon: float, d_east_m: float, d_north_m: float) -> Tuple[float, float]:
    """Convert local East-North metric offset (dE, dN) in meters from (ref_lat, ref_lon) into WGS84 (lat, lon)."""
    earth_radius_m = 6371000.0
    d_lat = math.degrees(d_north_m / earth_radius_m)
    lat_out = ref_lat + d_lat
    lat_avg_rad = math.radians((ref_lat + lat_out) * 0.5)
    d_lon = math.degrees(d_east_m / max(1e-6, earth_radius_m * math.cos(lat_avg_rad)))
    return lat_out, ref_lon + d_lon


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
        # DISTANCE SOURCE: 'auto' (RTK GPS outdoors -> LiDAR Odom fallback), 'gps', or 'odom'
        self.declare_parameter('distance_source', 'auto')
        self.declare_parameter('gps_require_rtk', False)        # False = allow 3D/DGPS/RTK in 'gps' mode; True = require RTK Fix/Float
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
        self.cmd_waypoint_sub = self.create_subscription(
            Vector3, '/cmd_waypoint', self._cmd_waypoint_callback, 10, callback_group=self.cb_group
        )

        # Sensor Subscriptions (6-DOF Gyro /imu/heading only — no magnetometer compass used)
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
        self.base_fix_sub = self.create_subscription(
            NavSatFix, '/gps/base_fix', self._base_fix_callback, 10, callback_group=self.cb_group
        )
        self.gps_status_sub = self.create_subscription(
            String, '/gps/status', self._gps_status_callback, 10, callback_group=self.cb_group
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

        # IMU State (6-DOF Gyro CCW+ radians, continuous unrolled yaw) & Global ENU Heading Fusion
        self.current_heading_rad: Optional[float] = None
        self.compass_initialized: bool = False
        self.enu_yaw_offset_rad: float = 0.0
        self.enu_yaw_locked_by_rtk: bool = False
        self._rtk_course_anchor_gps: Optional[Tuple[float, float]] = None
        self._rtk_course_anchor_yaw: float = 0.0
        self.unrolled_yaw_rad: float = 0.0
        self.start_unrolled_yaw: float = 0.0
        self.current_yaw_rate: float = 0.0
        self.last_heading_time: float = 0.0

        # Unified 2D Position & Distance State (Populated by either GPS or LiDAR Odom)
        self.active_sensor_mode: str = "NONE"   # "GPS" or "ODOM"
        self.chord_dist_m: float = 0.0          # Direct Euclidean distance from start (m)
        self.accumulated_path_dist_m: float = 0.0  # Incremental path length along curve/line (m)

        # Raw Odom Sensor State & Last-RTK-Fix Handover Snapshot
        self.current_odom_pos: Optional[Tuple[float, float]] = None
        self.start_odom_pos: Optional[Tuple[float, float]] = None
        self.last_odom_accum_pos: Optional[Tuple[float, float]] = None
        self.last_odom_time: float = 0.0
        self._odom_handover_dist_m: float = 0.0
        self._odom_pos_at_last_rtk_fix: Optional[Tuple[float, float]] = None
        self._dist_at_last_rtk_fix: float = 0.0
        self._accum_dist_at_last_rtk_fix: float = 0.0
        self._odom_accum_since_last_rtk_fix: float = 0.0

        # Raw GPS Sensor State, RTK Base (0, 0) Origin & 1 Hz Sub-Second Inter-Fix Predictor
        self.current_gps_coords: Optional[Tuple[float, float]] = None
        self.base_gps_coords: Optional[Tuple[float, float]] = None
        self.start_gps_coords: Optional[Tuple[float, float]] = None
        self.last_gps_accum_coords: Optional[Tuple[float, float]] = None
        self.waypoint_target_gps: Optional[Tuple[float, float]] = None
        self.waypoint_target_odom: Optional[Tuple[float, float]] = None
        self._goto_phase: str = "NONE"          # "ALIGN" or "DRIVE"
        self._goto_yaw_ref: float = 0.0
        self._min_rem_dist_wp: Optional[float] = None
        self._goto_arrival_reached: bool = False
        self.goal_start_time: float = 0.0
        self.goal_timeout_s: float = 60.0
        self.gps_status: int = NavSatStatus.STATUS_NO_FIX
        self.gps_is_rtk_fix: bool = False
        self.last_gps_time: float = 0.0
        self._gps_interfix_dist_m: float = 0.0
        self._gps_speed_scale: float = 1.32
        self._prev_gps_dist_m: float = 0.0
        self._prev_gps_fix_time: float = 0.0
        self._cmd_v_accum: float = 0.0
        self._gps_stop_wait_start: Optional[float] = None

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
        self.get_logger().info(" Heading Mode   : 6-DOF Gyro + Manual North + Driving GPS Fine-Cal")
        self.get_logger().info("==================================================")

    # --------------------------------------------------------------------------
    # Sensor Callbacks
    # --------------------------------------------------------------------------
    def _heading_callback(self, msg: Float32) -> None:
        """Converts [0, 360) deg CCW 6-DOF gyro heading into [-pi, pi] rad and unrolls continuous yaw."""
        rad = math.radians(normalize_angle_deg(msg.data))
        norm_rad = normalize_angle_rad(rad)

        if self.current_heading_rad is not None:
            diff = shortest_angular_diff_rad(norm_rad, self.current_heading_rad)
            self.unrolled_yaw_rad += diff
        else:
            self.unrolled_yaw_rad = norm_rad
            # Default boot orientation to North (+90° ENU) until user clicks "Set Rover = NORTH (90°)"
            if not self.compass_initialized:
                self.enu_yaw_offset_rad = shortest_angular_diff_rad(math.pi / 2.0, norm_rad)

        self.current_heading_rad = norm_rad
        self.last_heading_time = time.time()

    def _base_fix_callback(self, msg: NavSatFix) -> None:
        """Locks the RTK Base Station (0, 0) coordinate decoded from RTCM 1005/1006."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        b_lat = float(msg.latitude)
        b_lon = float(msg.longitude)
        if abs(b_lat) > 1e-4 or abs(b_lon) > 1e-4:
            self.base_gps_coords = (b_lat, b_lon)

    def _get_global_enu_yaw_rad(self) -> float:
        """Returns robot heading in global East-North-Up radians (0 = East, +pi/2 = North)."""
        if self.current_heading_rad is None:
            return 0.0
        return normalize_angle_rad(self.current_heading_rad + self.enu_yaw_offset_rad)

    def _imu_data_callback(self, msg: Imu) -> None:
        self.current_yaw_rate = float(msg.angular_velocity.z)

    def _odom_callback(self, msg: Odometry) -> None:
        curr_x = float(msg.pose.pose.position.x)
        curr_y = float(msg.pose.pose.position.y)
        self.current_odom_pos = (curr_x, curr_y)
        self.last_odom_time = time.time()

        if self.state == ControllerState.EXECUTING:
            if self.active_sensor_mode == "ODOM" and self.start_odom_pos is not None:
                if (
                    self.active_motion_type == "GOTO"
                    and self._goto_phase == "DRIVE"
                    and self.waypoint_target_odom is not None
                ):
                    rem_x = self.waypoint_target_odom[0] - curr_x
                    rem_y = self.waypoint_target_odom[1] - curr_y
                    rem_dist_wp = math.hypot(rem_x, rem_y)
                    self.chord_dist_m = max(0.0, abs(self.target_dist_m) - rem_dist_wp)
                    if rem_dist_wp > 0.18 and self.current_heading_rad is not None:
                        bearing_odom = math.atan2(rem_y, rem_x)
                        bearing_err = shortest_angular_diff_rad(bearing_odom, self.current_heading_rad)
                        self._goto_yaw_ref = self._get_measured_yaw_rad() + max(
                            -math.radians(20.0), min(math.radians(20.0), bearing_err)
                        )
                else:
                    self.chord_dist_m = self._odom_handover_dist_m + math.hypot(
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
                    if self.active_sensor_mode == "ODOM":
                        self.accumulated_path_dist_m += step_m
                    else:
                        self._odom_accum_since_last_rtk_fix += step_m
                    self.last_odom_accum_pos = (curr_x, curr_y)

    def _gps_status_callback(self, msg: String) -> None:
        text = msg.data
        if "Fix:" in text:
            fix_label = text.split("|")[0].replace("Fix:", "").strip().upper()
            self.gps_is_rtk_fix = (fix_label == "RTK FIX")

    def _fix_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.gps_status = int(msg.status.status)

        # Detect RTK FIX vs RTK FLOAT from covariance when /gps/status has not yet arrived
        cov0 = float(msg.position_covariance[0]) if len(msg.position_covariance) > 0 else 999.0
        if self.gps_status == NavSatStatus.STATUS_GBAS_FIX and 0.0 < cov0 <= 0.008:
            self.gps_is_rtk_fix = True
        elif self.gps_status != NavSatStatus.STATUS_GBAS_FIX or cov0 > 0.01:
            self.gps_is_rtk_fix = False

        # Ignore invalid / lost fix packets (STATUS_NO_FIX or (0.0, 0.0) coordinates)
        curr_lat = float(msg.latitude)
        curr_lon = float(msg.longitude)
        if self.gps_status == NavSatStatus.STATUS_NO_FIX or (abs(curr_lat) < 1e-4 and abs(curr_lon) < 1e-4):
            return

        now = time.time()
        self.current_gps_coords = (curr_lat, curr_lon)
        self.last_gps_time = now

        if self.state == ControllerState.EXECUTING and self.active_sensor_mode == "GPS":
            # In 'auto' mode, if GPS degrades from RTK FIX to RTK FLOAT while LiDAR /odom
            # is online, reject the noisy RTK FLOAT coordinate so handover stays anchored
            # to the exact last RTK FIX position + corresponding LiDAR scan pose.
            odom_online = (self.current_odom_pos is not None) and ((now - self.last_odom_time) < 1.5)
            if self.distance_source == 'auto' and not self.gps_is_rtk_fix and odom_online:
                return
            if not self._is_gps_fix_acceptable():
                return
            if self.start_gps_coords is not None:
                new_chord = gps_distance_m(
                    self.start_gps_coords[0], self.start_gps_coords[1],
                    curr_lat, curr_lon,
                )
                dt_fix = (now - self._prev_gps_fix_time) if self._prev_gps_fix_time > 0.0 else 0.0
                rem_to_target = abs(self.target_dist_m) - new_chord

                # Calibrate ground-to-command speed ratio strictly during steady cruising
                # (outside the final 85 cm braking ramp so deceleration/stopping never skews it)
                if (
                    0.40 <= dt_fix <= 2.0
                    and self._cmd_v_accum >= 0.12
                    and new_chord >= 0.30
                    and rem_to_target >= 0.85
                    and self._gps_stop_wait_start is None
                ):
                    delta_d = max(0.0, new_chord - self._prev_gps_dist_m)
                    if delta_d > 0.05:
                        raw_scale = max(0.85, min(1.85, delta_d / self._cmd_v_accum))
                        self._gps_speed_scale = 0.40 * self._gps_speed_scale + 0.60 * raw_scale

                self.chord_dist_m = new_chord
                self._prev_gps_dist_m = new_chord
                self._prev_gps_fix_time = now
                self._cmd_v_accum = 0.0

                # Snapshot the LiDAR /odom pose at this exact valid RTK FIX so any future
                # RTK FIX -> RTK FLOAT switch measures LiDAR odometry from this exact point.
                if self.gps_is_rtk_fix:
                    self._dist_at_last_rtk_fix = new_chord
                    self._odom_pos_at_last_rtk_fix = self.current_odom_pos
                    self._odom_accum_since_last_rtk_fix = 0.0

                # Driving Fine-Calibration of 6-DOF Gyro via GPS Course-Over-Ground (COG)
                # Runs ONLY while actively driving straight and within ±40° of the calibrated heading
                if (
                    self.active_motion_type in ["STRAIGHT", "GOTO"]
                    and self._goto_phase in ["NONE", "DRIVE"]
                    and self._rtk_course_anchor_gps is not None
                    and abs(self.last_cmd_v) > 0.05
                    and self.current_heading_rad is not None
                ):
                    step_e, step_n = gps_enu_delta_m(
                        self._rtk_course_anchor_gps[0], self._rtk_course_anchor_gps[1],
                        curr_lat, curr_lon,
                    )
                    step_dist = math.hypot(step_e, step_n)
                    yaw_drift = abs(self._get_measured_yaw_rad() - self._rtk_course_anchor_yaw)

                    if yaw_drift > math.radians(5.0) or step_dist > 5.0:
                        self._rtk_course_anchor_gps = (curr_lat, curr_lon)
                        self._rtk_course_anchor_yaw = self._get_measured_yaw_rad()
                    else:
                        min_step_m = 0.25 if self.gps_is_rtk_fix else 0.60
                        if step_dist >= min_step_m:
                            course_enu = math.atan2(step_n, step_e)
                            if self.last_cmd_v < 0.0:
                                course_enu = normalize_angle_rad(course_enu + math.pi)
                            meas_offset = shortest_angular_diff_rad(course_enu, self.current_heading_rad)
                            diff_off = shortest_angular_diff_rad(meas_offset, self.enu_yaw_offset_rad)

                            # Only fine-tune within ±40° of current calibrated heading so GPS noise cannot flip it
                            if abs(diff_off) <= math.radians(40.0):
                                alpha = 0.35 if (self.gps_is_rtk_fix and not self.enu_yaw_locked_by_rtk) else 0.20
                                self.enu_yaw_offset_rad = normalize_angle_rad(self.enu_yaw_offset_rad + alpha * diff_off)
                                self.enu_yaw_locked_by_rtk = True
                                self.get_logger().info(
                                    f"[GPS FINE-CAL] Refined ENU heading over {step_dist*100:.0f}cm -> "
                                    f"{math.degrees(self._get_global_enu_yaw_rad()):+.1f}° ENU "
                                    f"(adj: {math.degrees(alpha * diff_off):+.1f}°)"
                                )
                            if step_dist >= 1.0:
                                self._rtk_course_anchor_gps = (curr_lat, curr_lon)
                                self._rtk_course_anchor_yaw = self._get_measured_yaw_rad()

                # For GOTO waypoint navigation in DRIVE phase: continuously home onto exact target coordinate
                if (
                    self.active_motion_type == "GOTO"
                    and self._goto_phase == "DRIVE"
                    and self.waypoint_target_gps is not None
                ):
                    rem_e, rem_n = gps_enu_delta_m(
                        curr_lat, curr_lon,
                        self.waypoint_target_gps[0], self.waypoint_target_gps[1],
                    )
                    rem_dist_wp = math.hypot(rem_e, rem_n)
                    new_chord = max(0.0, abs(self.target_dist_m) - rem_dist_wp)
                    self.chord_dist_m = new_chord
                    self._prev_gps_dist_m = new_chord
                    rem_to_target = rem_dist_wp

                    # Track closest point of approach achieved during this drive
                    if self._min_rem_dist_wp is None or rem_dist_wp < self._min_rem_dist_wp:
                        self._min_rem_dist_wp = rem_dist_wp

                    # Precision Arrival / CPA (Closest Point of Approach) Detection:
                    # 1. Direct hit within tolerance
                    # 2. Reached closest point of approach within close range (<= 8 cm) and started increasing
                    is_direct_hit = rem_dist_wp <= self.distance_tolerance_m
                    is_cpa_reached = (
                        self._min_rem_dist_wp is not None
                        and self._min_rem_dist_wp <= 0.08
                        and rem_dist_wp >= (self._min_rem_dist_wp + 0.012)
                    )
                    if is_direct_hit or is_cpa_reached:
                        self._goto_arrival_reached = True

                    # Fine-steering homing down to 5 cm (gentle clamp inside 25 cm)
                    if rem_dist_wp > 0.05 and not self._goto_arrival_reached:
                        bearing_enu = math.atan2(rem_n, rem_e)
                        bearing_err = shortest_angular_diff_rad(bearing_enu, self._get_global_enu_yaw_rad())
                        if abs(bearing_err) > math.radians(35.0) and rem_dist_wp > 0.35:
                            # Large initial heading discrepancy: pivot in place to true bearing
                            self.get_logger().info(
                                f"[GOTO RE-ALIGN] Adjusting heading by {math.degrees(bearing_err):+.1f}° to true GPS bearing."
                            )
                            self._goto_phase = "ALIGN"
                            self.start_unrolled_yaw = self.unrolled_yaw_rad
                            self.target_yaw_rad = bearing_err
                            self.turn_settle_start = None
                            self.profiler.start_rotate(bearing_err, max_w=self.current_max_w)
                        else:
                            max_trim = math.radians(10.0) if rem_dist_wp <= 0.25 else math.radians(30.0)
                            clamped_err = max(-max_trim, min(max_trim, bearing_err))
                            self._goto_yaw_ref = self._get_measured_yaw_rad() + clamped_err

                # Account for ~0.42s LC29H RTK computation + NMEA serial pipeline latency
                # plus ~3.5 cm track spin-down lead while actively driving; reset to 0.0 when stopped.
                if self._gps_stop_wait_start is None and abs(self.last_cmd_v) > 0.01 and not self._goto_arrival_reached:
                    eff_cmd_v = abs(self.last_cmd_v) + 0.025
                    est_v = eff_cmd_v * self._gps_speed_scale
                    self._gps_interfix_dist_m = min(0.14, est_v * 0.42 + 0.035)
                else:
                    self._gps_interfix_dist_m = 0.0
                    # If this is a true post-stop stationary fix (>= 0.45s after motor cut)
                    # and the robot stopped slightly short of target, allow gentle creep only if CPA not yet reached
                    if (
                        self._gps_stop_wait_start is not None
                        and not self._goto_arrival_reached
                        and (now - self._gps_stop_wait_start) >= 0.45
                        and rem_to_target > self.distance_tolerance_m
                    ):
                        self._gps_stop_wait_start = None

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
                if self.gps_is_rtk_fix:
                    self._accum_dist_at_last_rtk_fix = self.accumulated_path_dist_m

    def _battery_callback(self, msg: BatteryState) -> None:
        if msg.voltage > 5.0:
            self.battery_voltage = float(msg.voltage)

    def _is_gps_fix_acceptable(self) -> bool:
        """Checks whether current GPS fix status meets configuration requirements."""
        if self.gps_require_rtk:
            return self.gps_status in [NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.STATUS_SBAS_FIX]
        return self.gps_status >= NavSatStatus.STATUS_FIX

    # --------------------------------------------------------------------------
    # Topic Command Callbacks (/cmd_step, /cmd_primitive, /cmd_waypoint)
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

    def _cmd_waypoint_callback(self, msg: Vector3) -> None:
        """
        Receives /cmd_waypoint:
          - If msg.z == 0.0: Base-relative ENU meters (msg.x = East_m, msg.y = North_m from Base 0,0)
          - If msg.z == 1.0: Global WGS84 degrees (msg.x = Latitude, msg.y = Longitude)
          - If msg.z == 2.0: Manual Heading Calibration (msg.x = target ENU degrees, e.g. 90.0 = North)
        """
        if abs(float(msg.z) - 2.0) < 0.1:
            target_enu_deg = float(msg.x)
            target_enu_rad = normalize_angle_rad(math.radians(target_enu_deg))
            if self.current_heading_rad is not None:
                self.enu_yaw_offset_rad = shortest_angular_diff_rad(target_enu_rad, self.current_heading_rad)
                self.compass_initialized = True
                self.enu_yaw_locked_by_rtk = False
                self._rtk_course_anchor_gps = self.current_gps_coords
                self._rtk_course_anchor_yaw = self._get_measured_yaw_rad()
                self.get_logger().info(
                    f"[HEADING CALIBRATED] Manual heading set to "
                    f"{math.degrees(self._get_global_enu_yaw_rad()) % 360.0:.1f}° ENU. "
                    "Driving GPS fine-calibration armed."
                )
                self._publish_diagnostics()
            else:
                self.get_logger().warn("[HEADING CALIBRATE] /imu/heading not yet received!")
            return

        if self._current_goal_handle is not None:
            self._preempted_by_topic = True
        if abs(float(msg.z) - 1.0) < 0.1:
            if self.current_gps_coords is None:
                self.get_logger().error("[WAYPOINT REJECTED] Current GPS position unknown!")
                return
            de_m, dn_m = gps_enu_delta_m(
                self.current_gps_coords[0], self.current_gps_coords[1],
                float(msg.x), float(msg.y),
            )
            self._start_motion("GOTO_REL_ENU", de_m, dn_m)
        else:
            self._start_motion("GOTO_BASE_XY", float(msg.x), float(msg.y))

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
        # Always allow TYPE_STOP to execute immediately
        if getattr(goal_request, 'motion_type', None) in [3, getattr(ExecuteMotion.Goal, 'TYPE_STOP', 3)]:
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
        self._publish_cmd_vel(0.0, 0.0)
        return CancelResponse.ACCEPT

    def _execute_action_callback(self, goal_handle):
        self._current_goal_handle = goal_handle
        self._cancel_requested = False
        self._preempted_by_topic = False
        goal = goal_handle.request

        # Resolve motion type from Action Constants (compatible with integer codes 0-5)
        if goal.motion_type in [0, getattr(ExecuteMotion.Goal, 'TYPE_DRIVE_STRAIGHT', 0)]:
            m_type = "STRAIGHT"
        elif goal.motion_type in [1, getattr(ExecuteMotion.Goal, 'TYPE_ROTATE_IN_PLACE', 1)]:
            m_type = "ROTATE"
        elif goal.motion_type in [2, getattr(ExecuteMotion.Goal, 'TYPE_DRIVE_ARC', 2)]:
            m_type = "ARC"
        elif goal.motion_type in [3, getattr(ExecuteMotion.Goal, 'TYPE_STOP', 3)]:
            m_type = "STOP"
        elif goal.motion_type in [4, getattr(ExecuteMotion.Goal, 'TYPE_GOTO_BASE_XY', 4)]:
            m_type = "GOTO_BASE_XY"
        elif goal.motion_type in [5, getattr(ExecuteMotion.Goal, 'TYPE_GOTO_REL_ENU', 5)]:
            m_type = "GOTO_REL_ENU"
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

            measured_dist = self._get_measured_distance_m(include_interfix=True)
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

        if self._cancel_requested or goal_handle.is_cancel_requested:
            self._stop_motion()
            goal_handle.canceled()
            result.success = False
            result.error_code = ExecuteMotion.Result.ERROR_CANCELED
            result.message = "Motion canceled by client"
            self._current_goal_handle = None
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
        gps_rtk_fix_fresh = (
            (self.current_gps_coords is not None)
            and (now - self.last_gps_time < 2.0)
            and self.gps_is_rtk_fix
        )

        # Explicit GPS mode (GPS-only preset)
        if self.distance_source == 'gps':
            if not gps_fresh:
                req_str = "RTK Fix/Float" if self.gps_require_rtk else "valid 3D/RTK Fix"
                return False, f"GPS (/fix) not ready or missing {req_str} (status={self.gps_status})!"
            self.active_sensor_mode = "GPS"
            return True, "GPS (/fix)"

        # Explicit ODOM mode (LiDAR-only preset)
        if self.distance_source == 'odom':
            if not odom_fresh:
                return False, "LiDAR Odometry (/odom) is not publishing!"
            self.active_sensor_mode = "ODOM"
            return True, "ODOM (/odom)"

        # Auto-Priority mode (Dual GPS + LiDAR Robot):
        # 1. Prefer RTK GPS outdoors when RTK FIX is locked
        # 2. Automatically fall back to LiDAR /odom if GPS degrades or indoors
        # 3. If LiDAR /odom is offline, use GPS if any acceptable fix is present
        if gps_rtk_fix_fresh:
            self.active_sensor_mode = "GPS"
            return True, "GPS (RTK FIX - Auto Priority)"
        if odom_fresh:
            self.active_sensor_mode = "ODOM"
            return True, "ODOM (LiDAR Auto-Fallback)"
        if gps_fresh:
            self.active_sensor_mode = "GPS"
            return True, "GPS (Fallback - Odom Offline)"

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

        needs_distance = motion_type in ["STRAIGHT", "ARC", "GOTO_BASE_XY", "GOTO_REL_ENU", "GOTO"]
        sensor_ok, sensor_desc = self._resolve_active_sensor(needs_distance)
        if not sensor_ok:
            self.get_logger().error(f"[START REJECTED] {sensor_desc}")
            self._set_state(ControllerState.SENSOR_ERROR, sensor_desc)
            return False, sensor_desc

        self.current_max_v = max_v if (max_v and max_v > 0.0) else self.default_linear_speed
        self.current_max_w = max_w if (max_w and max_w > 0.0) else self.default_angular_speed
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.yaw_integral = 0.0
        self.turn_settle_start = None
        self._goto_phase = "NONE"
        self._goto_yaw_ref = 0.0
        self._min_rem_dist_wp = None
        self._goto_arrival_reached = False
        self.goal_start_time = time.time()
        if motion_type == "ROTATE":
            self.goal_timeout_s = max(12.0, (abs(yaw_rad) / max(0.3, self.min_angular_speed)) * 2.5 + 6.0)
        else:
            self.goal_timeout_s = max(18.0, (abs(dist_m) / max(0.08, self.min_linear_speed)) * 2.5 + 10.0)
        self.waypoint_target_gps = None
        self.waypoint_target_odom = None

        # Resolve Coordinate Waypoint Goals (Base-relative X,Y or Global/Relative dE,dN)
        if motion_type in ["GOTO_BASE_XY", "GOTO_REL_ENU"]:
            if self.active_sensor_mode == "GPS":
                if self.current_gps_coords is None:
                    return False, "Current GPS position is unknown!"
                curr_lat, curr_lon = self.current_gps_coords
                if motion_type == "GOTO_BASE_XY":
                    if self.base_gps_coords is None:
                        return False, "RTK Base Station (0, 0) coordinate not yet received on /gps/base_fix!"
                    t_lat, t_lon = gps_enu_to_latlon(
                        self.base_gps_coords[0], self.base_gps_coords[1],
                        float(dist_m), float(yaw_rad),
                    )
                    self.waypoint_target_gps = (t_lat, t_lon)
                    rem_e, rem_n = gps_enu_delta_m(curr_lat, curr_lon, t_lat, t_lon)
                else:  # GOTO_REL_ENU (dEast_m, dNorth_m computed from 64-bit Global Lat/Lon)
                    rem_e, rem_n = float(dist_m), float(yaw_rad)
                    t_lat, t_lon = gps_enu_to_latlon(curr_lat, curr_lon, rem_e, rem_n)
                    self.waypoint_target_gps = (t_lat, t_lon)
            else:  # ODOM mode
                if self.current_odom_pos is None:
                    return False, "Current LiDAR /odom position is unknown!"
                curr_x, curr_y = self.current_odom_pos
                if motion_type == "GOTO_BASE_XY":
                    self.waypoint_target_odom = (float(dist_m), float(yaw_rad))
                    rem_e = float(dist_m) - curr_x
                    rem_n = float(yaw_rad) - curr_y
                else:
                    rem_e, rem_n = float(dist_m), float(yaw_rad)
                    self.waypoint_target_odom = (curr_x + rem_e, curr_y + rem_n)

            wp_dist = math.hypot(rem_e, rem_n)
            desired_enu_yaw = math.atan2(rem_n, rem_e)
            ref_yaw = (
                self._get_global_enu_yaw_rad()
                if self.active_sensor_mode == "GPS"
                else (self.current_heading_rad or 0.0)
            )
            turn_delta = shortest_angular_diff_rad(desired_enu_yaw, ref_yaw)
            motion_type = "GOTO"
            dist_m = wp_dist
            yaw_rad = turn_delta

        self.active_motion_type = motion_type
        self.target_dist_m = float(dist_m)
        self.target_yaw_rad = float(yaw_rad)

        # Lock IMU Heading Baseline
        self.start_unrolled_yaw = self.unrolled_yaw_rad
        self._rtk_course_anchor_gps = self.current_gps_coords
        self._rtk_course_anchor_yaw = 0.0

        # Lock 2D Position & Path Distance Baselines
        self.chord_dist_m = 0.0
        self.accumulated_path_dist_m = 0.0
        self._odom_handover_dist_m = 0.0
        self._odom_pos_at_last_rtk_fix = self.current_odom_pos
        self._dist_at_last_rtk_fix = 0.0
        self._accum_dist_at_last_rtk_fix = 0.0
        self._odom_accum_since_last_rtk_fix = 0.0
        self._gps_interfix_dist_m = 0.0
        self._gps_speed_scale = 1.32
        self._prev_gps_dist_m = 0.0
        self._prev_gps_fix_time = time.time()
        self._cmd_v_accum = 0.0
        self._gps_stop_wait_start = None
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
        elif motion_type == "GOTO":
            if abs(dist_m) <= self.distance_tolerance_m:
                self._goto_phase = "DRIVE"
                self.profiler.start_straight(0.0, max_v=self.current_max_v)
            elif abs(yaw_rad) > self.yaw_tolerance_rad * 1.5:
                self._goto_phase = "ALIGN"
                self.profiler.start_rotate(yaw_rad, max_w=self.current_max_w)
            else:
                self._goto_phase = "DRIVE"
                self._goto_yaw_ref = yaw_rad
                self.target_yaw_rad = 0.0
                self.profiler.start_straight(dist_m, max_v=self.current_max_v)

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
        if self.state != ControllerState.EXECUTING or self._cancel_requested:
            return

        now = time.time()
        dt = 1.0 / self.control_rate_hz

        # Seamless mid-maneuver automatic fallback from RTK GPS to LiDAR /odom in 'auto' mode
        # if RTK FIX degrades (e.g. to RTK FLOAT) or GPS times out while LiDAR odometry is online.
        # Anchors directly to the LiDAR scan pose captured at the LAST valid RTK FIX!
        if (
            self.distance_source == 'auto'
            and self.active_sensor_mode == "GPS"
            and self.active_motion_type in ["STRAIGHT", "ARC", "GOTO"]
            and (not self.gps_is_rtk_fix or (now - self.last_gps_time) > 2.0)
            and self.current_odom_pos is not None
            and (now - self.last_odom_time) < 1.5
        ):
            anchor_odom = self._odom_pos_at_last_rtk_fix or self.start_odom_pos or self.current_odom_pos
            self._odom_handover_dist_m = self._dist_at_last_rtk_fix
            self.start_odom_pos = anchor_odom
            odom_delta_since_fix = math.hypot(
                self.current_odom_pos[0] - anchor_odom[0],
                self.current_odom_pos[1] - anchor_odom[1],
            )
            self.chord_dist_m = self._odom_handover_dist_m + odom_delta_since_fix
            self.accumulated_path_dist_m = self._accum_dist_at_last_rtk_fix + max(
                self._odom_accum_since_last_rtk_fix, odom_delta_since_fix
            )
            self._gps_interfix_dist_m = 0.0
            self._gps_stop_wait_start = None
            self.active_sensor_mode = "ODOM"
            self.get_logger().warn(
                f"[AUTO FALLBACK] GPS RTK Fix degraded -> Anchored to last RTK FIX "
                f"({self._dist_at_last_rtk_fix:.2f}m) + LiDAR delta since fix "
                f"(+{odom_delta_since_fix:.2f}m = {self.chord_dist_m:.2f}m)!"
            )

        # Update sub-second inter-fix displacement between 1 Hz RTK GPS updates
        # (Strictly bounded to a single 1s GPS interval; resets on every /fix)
        if (
            self.active_sensor_mode == "GPS"
            and (
                self.active_motion_type in ["STRAIGHT", "ARC"]
                or (self.active_motion_type == "GOTO" and self._goto_phase == "DRIVE")
            )
            and self._gps_stop_wait_start is None
            and (now - self.last_gps_time) <= 1.05
            and abs(self.last_cmd_v) > 0.01
        ):
            eff_cmd_v = abs(self.last_cmd_v) + 0.025
            est_ground_v = eff_cmd_v * self._gps_speed_scale
            self._gps_interfix_dist_m = min(0.40, self._gps_interfix_dist_m + est_ground_v * dt)
            self._cmd_v_accum += eff_cmd_v * dt

        # 1. Measure actual physical distance and heading change
        actual_dist = self._get_measured_distance_m(include_interfix=True)
        true_sensor_dist = self._get_measured_distance_m(include_interfix=False)
        actual_yaw = self._get_measured_yaw_rad()

        # 2. Sample desired velocity & heading reference from MEASURED STATE (not time!)
        v_ref, omega_ref, _s_ref, yaw_ref, dist_or_rot_done = self.profiler.sample_by_state(
            actual_dist_m=actual_dist,
            actual_yaw_rad=actual_yaw,
            dt=dt,
            dist_tolerance_m=self.distance_tolerance_m,
            yaw_tolerance_rad=self.yaw_tolerance_rad,
        )
        if self._gps_stop_wait_start is not None or self._goto_arrival_reached:
            dist_or_rot_done = True

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
            (
                (self.active_motion_type == "ROTATE" or (self.active_motion_type == "GOTO" and self._goto_phase == "ALIGN"))
                and abs(yaw_error_to_target) <= math.radians(4.0)
            )
            or (self.active_motion_type in ["STRAIGHT", "ARC", "GOTO"] and dist_remaining <= 0.20)
        )

        # Global Action Timeout Watchdog (prevents runaway if goal is unreachable)
        if (now - self.goal_start_time) > self.goal_timeout_s:
            self.get_logger().error(
                f"[GOAL TIMEOUT] Motion exceeded deadline of {self.goal_timeout_s:.1f}s! Stopping robot."
            )
            self._publish_cmd_vel(0.0, 0.0)
            self._set_state(ControllerState.STUCK, "GOAL TIMEOUT")
            return

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
        # MODE A: ROTATE IN PLACE (or GOTO Initial/Mid Alignment Phase)
        # ----------------------------------------------------------------------
        if self.active_motion_type == "ROTATE" or (self.active_motion_type == "GOTO" and self._goto_phase == "ALIGN"):
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
                    if self.active_motion_type == "GOTO":
                        # Transition from ALIGN phase into DRIVE phase toward the waypoint
                        self._goto_phase = "DRIVE"
                        self.turn_settle_start = None
                        self.start_unrolled_yaw = self.unrolled_yaw_rad
                        self._goto_yaw_ref = 0.0
                        self.target_yaw_rad = 0.0
                        self.yaw_integral = 0.0
                        self.chord_dist_m = 0.0
                        self._min_rem_dist_wp = None
                        self._goto_arrival_reached = False
                        self.goal_start_time = now
                        self._gps_interfix_dist_m = 0.0
                        self._prev_gps_dist_m = 0.0
                        self._prev_gps_fix_time = now
                        self._cmd_v_accum = 0.0
                        self.start_gps_coords = self.current_gps_coords
                        self.last_gps_accum_coords = self.current_gps_coords
                        self.start_odom_pos = self.current_odom_pos
                        self.last_odom_accum_pos = self.current_odom_pos
                        self._rtk_course_anchor_gps = self.current_gps_coords
                        self._rtk_course_anchor_yaw = 0.0
                        if self.active_sensor_mode == "GPS" and self.waypoint_target_gps and self.current_gps_coords:
                            re, rn = gps_enu_delta_m(
                                self.current_gps_coords[0], self.current_gps_coords[1],
                                self.waypoint_target_gps[0], self.waypoint_target_gps[1],
                            )
                            self.target_dist_m = math.hypot(re, rn)
                        self.goal_timeout_s = max(18.0, (abs(self.target_dist_m) / max(0.08, self.min_linear_speed)) * 2.5 + 10.0)
                        self.profiler.start_straight(self.target_dist_m, max_v=self.current_max_v)
                        self.get_logger().info(
                            f"[GOTO ALIGNED] Heading aligned -> Driving {self.target_dist_m:.2f}m to waypoint."
                        )
                    else:
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
            # On 1 Hz RTK GPS, cut motors immediately when sub-second target is reached
            # and wait at standstill for a post-stop stationary 1 Hz /fix (>= 0.45s after
            # motor cutoff) to confirm true resting position before completing.
            if self.active_sensor_mode == "GPS" and not is_full_circle:
                if self._gps_stop_wait_start is None:
                    self._gps_stop_wait_start = now
                self._publish_cmd_vel(0.0, 0.0)
                has_stationary_fix = (self.last_gps_time - self._gps_stop_wait_start) >= 0.45
                wait_elapsed = now - self._gps_stop_wait_start
                if not has_stationary_fix and wait_elapsed < 1.55:
                    return
                if (abs(self.target_dist_m) - true_sensor_dist) > self.distance_tolerance_m and wait_elapsed < 1.55:
                    return

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
                f"[{self.active_motion_type} COMPLETED] Dist: {true_sensor_dist:.3f}m "
                f"(Target: {abs(self.target_dist_m):.2f}m) | "
                f"Yaw: {math.degrees(actual_yaw):+.1f}° (Target: {math.degrees(self.target_yaw_rad):+.1f}°)"
            )
            return

        # 50 Hz Closed-Loop IMU Heading & Curvature Controller
        # For STRAIGHT: yaw_ref = 0.0 (locks initial heading).
        # For GOTO:     yaw_ref = self._goto_yaw_ref (continuously homes onto waypoint).
        # For ARC:      yaw_ref = (s / |X|) * Y (tracks exact geometric arc curvature).
        if self.active_motion_type == "GOTO":
            yaw_ref = self._goto_yaw_ref
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

    def _get_measured_distance_m(self, include_interfix: bool = False) -> float:
        """
        Returns physical ground distance traveled in meters from the active position sensor
        (GPS or LiDAR Odom). Never uses open-loop elapsed time.
        """
        if self.active_motion_type == "ROTATE":
            return 0.0

        interfix = (
            self._gps_interfix_dist_m
            if (include_interfix and self.active_sensor_mode == "GPS")
            else 0.0
        )

        if self.active_motion_type == "STRAIGHT":
            return self.chord_dist_m + interfix

        if self.active_motion_type == "ARC":
            # For arcs <= 125 deg, convert direct chord displacement into exact arc length
            # using measured heading change: s = chord * (|yaw|/2) / sin(|yaw|/2).
            # This is immune to incremental jitter while accurately measuring curved path length.
            abs_yaw = abs(self._get_measured_yaw_rad())
            if abs(self.target_yaw_rad) <= math.radians(125.0) and abs_yaw <= math.radians(135.0):
                if abs_yaw > 0.05:
                    half_yaw = 0.5 * abs_yaw
                    arc_factor = half_yaw / max(1e-4, math.sin(half_yaw))
                    base_arc = max(self.chord_dist_m * arc_factor, self.accumulated_path_dist_m)
                    return base_arc + interfix
                return self.chord_dist_m + interfix
            # For large arcs (> 125 deg / U-turns / full circles), use incremental path odometer
            return max(self.accumulated_path_dist_m, self.chord_dist_m) + interfix

        return self.chord_dist_m + interfix

    def _publish_cmd_vel(self, vx: float, wz: float) -> None:
        """Sends velocity target to the motor controller node."""
        self.last_cmd_v = float(vx)
        self.last_cmd_w = float(wz)
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

    def _publish_diagnostics(self) -> None:
        """Publishes lightweight Pi 5 SoC diagnostics and calibrated ENU gyro offset."""
        try:
            temp_c = None
            if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    val = f.read().strip()
                    if val:
                        temp_c = round(float(val) / 1000.0, 1)

            now = time.time()
            if self.state != ControllerState.EXECUTING:
                odom_ok = (self.current_odom_pos is not None) and (now - self.last_odom_time < 1.5)
                rtk_ok = (self.current_gps_coords is not None) and (now - self.last_gps_time < 2.0) and self.gps_is_rtk_fix
                gps_ok = (self.current_gps_coords is not None) and (now - self.last_gps_time < 2.0) and self._is_gps_fix_acceptable()
                if self.distance_source == 'gps':
                    sensor_label = "GPS" if gps_ok else "GPS (WAIT)"
                elif self.distance_source == 'odom':
                    sensor_label = "ODOM" if odom_ok else "ODOM (WAIT)"
                else:
                    if rtk_ok:
                        sensor_label = "AUTO: GPS (RTK)"
                    elif odom_ok:
                        sensor_label = "AUTO: ODOM"
                    elif gps_ok:
                        sensor_label = "AUTO: GPS"
                    else:
                        sensor_label = "AUTO (WAIT)"
            else:
                sensor_label = self.active_sensor_mode

            diag = {
                "cpu_temp": f"{temp_c:.1f} °C" if temp_c else "N/A",
                "battery_v": f"{self.battery_voltage:.2f} V",
                "state": self.state,
                "motion_type": self.active_motion_type,
                "sensor_mode": sensor_label,
                "enu_yaw_offset_deg": round(math.degrees(self.enu_yaw_offset_rad), 2),
                "enu_heading_deg": round(math.degrees(self._get_global_enu_yaw_rad()) % 360.0, 2),
                "enu_yaw_locked_by_rtk": bool(self.enu_yaw_locked_by_rtk),
                "compass_initialized": bool(self.compass_initialized),
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
