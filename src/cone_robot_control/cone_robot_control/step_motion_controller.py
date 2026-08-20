#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: Step Motion Controller (Turn X° and Drive Y cm)
==============================================================================
Description:
    Translates discrete high-level step commands (/cmd_step) into smooth,
    closed-loop motor velocity commands (/cmd_vel).
    
    Supports both GPS and LiDAR robot configurations:
      - Phase 1 (Turn X°): Closed-loop pivot rotation using BNO08x IMU heading.
      - Phase 2 (Drive Y cm): Closed-loop straight-line drive with active IMU
        yaw-correction and multi-sensor distance tracking:
          * Mode A: 2D Odometry (/odom) for odometry/LiDAR-equipped robots.
          * Mode B: RTK GPS displacement (/fix) for RTK-equipped robots.
          * Mode C: Calibrated speed-time integration fallback.

Subscribes:
    - /cmd_step (geometry_msgs/msg/Vector3): x = distance in cm, z = angle in deg.
    - /imu/heading (std_msgs/msg/Float32): 0° to 360° heading from BNO08x IMU.
    - /odom (nav_msgs/msg/Odometry): 2D Odometry (e.g. from laser/wheel odometry).
    - /fix (sensor_msgs/msg/NavSatFix): High-precision RTK GPS position.

Publishes:
    - /cmd_vel (geometry_msgs/msg/Twist): Continuous velocity sent to MDD10 motor controller.
    - /step_status (std_msgs/msg/String): Current state (IDLE, TURNING, DRIVING, COMPLETED).

Author: ConeRobot Team
License: MIT
==============================================================================
"""

import os
import math
import time
import json
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String

try:
    from nav_msgs.msg import Odometry
    NAV_MSGS_AVAILABLE = True
except ImportError:
    NAV_MSGS_AVAILABLE = False
    Odometry = None


def normalize_angle_deg(deg: float) -> float:
    """Normalize angle to [0.0, 360.0) degrees."""
    return deg % 360.0


def shortest_angular_diff_deg(target_deg: float, current_deg: float) -> float:
    """
    Calculate shortest signed angular difference (target - current) in degrees.
    Result is in [-180.0, 180.0]. Positive = turn right (clockwise).
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


class MotionState:
    IDLE = "IDLE"
    TURNING = "TURNING"
    DRIVING = "DRIVING"
    COMPLETED = "COMPLETED"


class StepMotionController(Node):
    """
    Two-phase discrete motion controller (Turn X deg -> Drive Y cm).
    """

    def __init__(self) -> None:
        super().__init__('step_motion_controller')

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter('control_rate_hz', 20.0)

        # Turning Parameters (BNO08x IMU)
        self.declare_parameter('turn_kp', 0.04)             # Proportional gain for turning
        self.declare_parameter('turn_tolerance_deg', 1.5)   # Target heading tolerance in degrees
        self.declare_parameter('min_turn_speed', 0.25)      # Minimum angular speed to prevent motor stall (rad/s)
        self.declare_parameter('max_turn_speed', 1.8)       # Maximum turning angular speed (rad/s)
        self.declare_parameter('turn_settle_time_s', 0.25)  # Required steady duration at target angle

        # Linear Driving Parameters
        self.declare_parameter('default_linear_speed', 0.25) # Cruising speed in m/s
        self.declare_parameter('yaw_lock_kp', 0.05)         # IMU straight-line correction gain
        self.declare_parameter('calibrated_speed_mps', 0.25)# Speed used for time-integration fallback

        # Multi-Sensor Distance Tracking Configuration
        self.declare_parameter('distance_source', 'auto')   # 'auto', 'odom', 'gps', 'time'
        self.declare_parameter('gps_require_rtk', True)     # Only use GPS if RTK Fix/Float is active

        # Mock / Simulation Mode
        self.declare_parameter('mock_mode', False)

        # --- Read Parameters ---
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.turn_kp = self.get_parameter('turn_kp').value
        self.turn_tolerance_deg = self.get_parameter('turn_tolerance_deg').value
        self.min_turn_speed = self.get_parameter('min_turn_speed').value
        self.max_turn_speed = self.get_parameter('max_turn_speed').value
        self.turn_settle_time_s = self.get_parameter('turn_settle_time_s').value

        self.default_linear_speed = self.get_parameter('default_linear_speed').value
        self.yaw_lock_kp = self.get_parameter('yaw_lock_kp').value
        self.calibrated_speed_mps = self.get_parameter('calibrated_speed_mps').value

        self.distance_source = self.get_parameter('distance_source').value
        self.gps_require_rtk = self.get_parameter('gps_require_rtk').value
        self.mock_mode = self.get_parameter('mock_mode').value

        # --- Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/step_status', 10)
        self.diag_pub = self.create_publisher(String, '/robot/diagnostics', 10)

        # 2.0s Lightweight SoC Diagnostics Timer (0 overhead)
        self.diag_timer = self.create_timer(2.0, self._publish_diagnostics)

        self.cmd_step_sub = self.create_subscription(Vector3, '/cmd_step', self._cmd_step_callback, 10)
        self.heading_sub = self.create_subscription(Float32, '/imu/heading', self._heading_callback, 10)
        if NAV_MSGS_AVAILABLE:
            self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        else:
            self.odom_sub = None
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self._fix_callback, 10)

        # --- Internal State ---
        self.state = MotionState.IDLE
        self.current_heading: Optional[float] = None
        self.last_heading_time: float = 0.0

        # Odometry state
        self.current_odom_pos: Optional[Tuple[float, float]] = None
        self.start_odom_pos: Optional[Tuple[float, float]] = None
        self.last_odom_time: float = 0.0

        # GPS state
        self.current_gps_coords: Optional[Tuple[float, float]] = None
        self.start_gps_coords: Optional[Tuple[float, float]] = None
        self.gps_status: int = NavSatStatus.STATUS_NO_FIX
        self.last_gps_time: float = 0.0

        # Active Goal
        self.target_turn_deg: float = 0.0       # Relative turn angle (+ right, - left)
        self.target_drive_cm: float = 0.0       # Relative distance (+ forward, - reverse)
        self.target_absolute_heading: float = 0.0
        self.heading_integral: float = 0.0      # PI controller error accumulator for motor trimming
        self.turn_start_time: Optional[float] = None
        self.turn_settle_start: Optional[float] = None
        self.drive_start_time: Optional[float] = None
        self.active_distance_method: str = "none"

        # Start 20 Hz Control Loop Timer
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info("==================================================")
        self.get_logger().info(" Step Motion Controller Node Initialized")
        self.get_logger().info(f" Control Rate : {self.control_rate_hz} Hz")
        self.get_logger().info(f" Dist Source  : {self.distance_source}")
        self.get_logger().info("==================================================")

    # --------------------------------------------------------------------------
    # Subscriber Callbacks
    # --------------------------------------------------------------------------
    def _cmd_step_callback(self, msg: Vector3) -> None:
        """
        Receives new motion step command:
          - msg.x: Distance in centimeters (+ forward, - backward)
          - msg.z: Turn angle in degrees (+ clockwise/right, - counter-clockwise/left)
        """
        if self.current_heading is None and not self.mock_mode:
            self.get_logger().warn("[STEP] Cannot start motion: waiting for first /imu/heading data!")
            return

        dist_cm = msg.x
        turn_deg = msg.z

        self.get_logger().info(
            f"[STEP RECEIVED] Turn: {turn_deg:+.1f}°, Drive: {dist_cm:+.1f} cm"
        )

        # Cancel any current motion and brake
        self._publish_cmd_vel(0.0, 0.0)

        # Set Targets
        self.target_turn_deg = turn_deg
        self.target_drive_cm = dist_cm
        start_heading = self.current_heading if self.current_heading is not None else 0.0
        self.target_absolute_heading = normalize_angle_deg(start_heading + turn_deg)
        self.turn_settle_start = None

        # Determine Initial Phase
        if abs(turn_deg) >= self.turn_tolerance_deg:
            self.turn_start_time = time.time()
            self._set_state(MotionState.TURNING)
            self.get_logger().info(
                f"[PHASE 1: TURN] Rotating from {start_heading:.1f}° to {self.target_absolute_heading:.1f}° "
                f"(Delta: {turn_deg:+.1f}°)"
            )
        elif abs(dist_cm) > 0.5:
            self._start_driving_phase()
        else:
            self.get_logger().info("[STEP] Zero movement requested. Returning to IDLE.")
            self._set_state(MotionState.COMPLETED)
            self._set_state(MotionState.IDLE)

    def _heading_callback(self, msg: Float32) -> None:
        self.current_heading = normalize_angle_deg(msg.data)
        self.last_heading_time = time.time()

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_odom_time = time.time()

    def _fix_callback(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.current_gps_coords = (msg.latitude, msg.longitude)
            self.gps_status = msg.status.status
            self.last_gps_time = time.time()

    # --------------------------------------------------------------------------
    # Main Control Loop (20 Hz)
    # --------------------------------------------------------------------------
    def _control_loop(self) -> None:
        if self.state == MotionState.IDLE or self.state == MotionState.COMPLETED:
            return

        now = time.time()

        # -----------------
        # 1. TURNING PHASE
        # -----------------
        if self.state == MotionState.TURNING:
            curr_h = self.current_heading if self.current_heading is not None else 0.0
            error_deg = shortest_angular_diff_deg(self.target_absolute_heading, curr_h)
            turn_duration = (now - self.turn_start_time) if self.turn_start_time else 0.0

            # Complete if within tolerance OR if close (<= 2.5 deg) and turning has stabilized for > 1.5s
            if abs(error_deg) <= self.turn_tolerance_deg or (turn_duration > 1.5 and abs(error_deg) <= 3.0):
                # Settle timer to ensure robot comes to a steady standstill
                if self.turn_settle_start is None:
                    self.turn_settle_start = now

                self._publish_cmd_vel(0.0, 0.0)

                if (now - self.turn_settle_start) >= self.turn_settle_time_s:
                    self.get_logger().info(
                        f"[TURN COMPLETED] Target: {self.target_absolute_heading:.1f}°, Final: {curr_h:.1f}° (Error: {error_deg:+.1f}°)"
                    )
                    # Proceed to Driving Phase or Complete
                    if abs(self.target_drive_cm) > 0.5:
                        self._start_driving_phase()
                    else:
                        self._set_state(MotionState.COMPLETED)
                        self._publish_cmd_vel(0.0, 0.0)
                        self._set_state(MotionState.IDLE)
            else:
                self.turn_settle_start = None
                # Proportional turn velocity: positive error (CCW/Left) -> positive wz (Left)
                raw_wz = self.turn_kp * error_deg

                # Clamp with minimum stall-prevention velocity
                sign = 1.0 if raw_wz >= 0 else -1.0
                clamped_wz = sign * max(self.min_turn_speed, min(self.max_turn_speed, abs(raw_wz)))
                self._publish_cmd_vel(0.0, clamped_wz)

        # -----------------
        # 2. DRIVING PHASE
        # -----------------
        elif self.state == MotionState.DRIVING:
            distance_traveled_cm = self._get_distance_traveled_cm()
            remaining_cm = abs(self.target_drive_cm) - distance_traveled_cm

            if remaining_cm <= 0.0:
                self.get_logger().info(
                    f"[DRIVE COMPLETED] Traveled {distance_traveled_cm:.1f} cm (Method: {self.active_distance_method})"
                )
                self._publish_cmd_vel(0.0, 0.0)
                self._set_state(MotionState.COMPLETED)
                self._set_state(MotionState.IDLE)
                return

            # Linear Speed Direction (+ forward, - reverse)
            direction_sign = 1.0 if self.target_drive_cm >= 0 else -1.0
            vx = direction_sign * self.default_linear_speed

            # Active IMU Straight-Line Yaw Lock with PI Controller (Standard ROS 2 CCW convention)
            if self.current_heading is not None and abs(self.target_turn_deg) < self.turn_tolerance_deg:
                heading_drift = shortest_angular_diff_deg(self.target_absolute_heading, self.current_heading)
                
                # Accumulate integral error with anti-windup (+/- 10 deg*s)
                dt = 1.0 / self.control_rate_hz
                self.heading_integral = max(-10.0, min(10.0, self.heading_integral + (heading_drift * dt)))
                
                # PI control: positive drift (target is to the left) -> steer left (+wz)
                p_term = self.yaw_lock_kp * heading_drift
                i_term = (self.yaw_lock_kp * 0.2) * self.heading_integral
                raw_yaw = p_term + i_term
                
                # Authority clamp to +/- 0.40 rad/s
                yaw_correction = max(-0.40, min(0.40, raw_yaw))
            else:
                yaw_correction = 0.0

            self._publish_cmd_vel(vx, yaw_correction)

    # --------------------------------------------------------------------------
    # Helper Functions
    # --------------------------------------------------------------------------
    def _start_driving_phase(self) -> None:
        """Initializes sensor baselines for the straight-line drive phase."""
        self._set_state(MotionState.DRIVING)
        self.drive_start_time = time.time()
        self.heading_integral = 0.0

        # Lock straight-line heading to current IMU heading at exact moment driving starts
        if abs(self.target_turn_deg) < self.turn_tolerance_deg and self.current_heading is not None:
            self.target_absolute_heading = self.current_heading

        # Capture Odometry baseline if recent (< 0.5s)
        if self.current_odom_pos and (time.time() - self.last_odom_time < 0.5):
            self.start_odom_pos = self.current_odom_pos
        else:
            self.start_odom_pos = None

        # Capture GPS baseline if recent (< 1.0s) and RTK mode condition is satisfied
        if self.current_gps_coords and (time.time() - self.last_gps_time < 1.0):
            if not self.gps_require_rtk or self.gps_status in [NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.STATUS_SBAS_FIX]:
                self.start_gps_coords = self.current_gps_coords
            else:
                self.start_gps_coords = None
        else:
            self.start_gps_coords = None

        # Determine Primary Method
        if (self.distance_source == 'odom' or self.distance_source == 'auto') and self.start_odom_pos:
            self.active_distance_method = "ODOM"
        elif (self.distance_source == 'gps' or self.distance_source == 'auto') and self.start_gps_coords:
            self.active_distance_method = "RTK_GPS"
        else:
            self.active_distance_method = "TIME_INTEGRATION"

        self.get_logger().info(
            f"[PHASE 2: DRIVE] Target: {self.target_drive_cm:+.1f} cm | Distance Method: [{self.active_distance_method}]"
        )

    def _get_distance_traveled_cm(self) -> float:
        """Returns distance traveled in cm from the active sensor."""
        now = time.time()

        # 1. 2D Odometry (LiDAR / Wheel Odometry Robot)
        if self.active_distance_method == "ODOM" and self.start_odom_pos and self.current_odom_pos:
            dx = self.current_odom_pos[0] - self.start_odom_pos[0]
            dy = self.current_odom_pos[1] - self.start_odom_pos[1]
            return math.sqrt(dx * dx + dy * dy) * 100.0

        # 2. RTK GPS (GPS Robot)
        if self.active_distance_method == "RTK_GPS" and self.start_gps_coords and self.current_gps_coords:
            dist_m = gps_distance_m(
                self.start_gps_coords[0], self.start_gps_coords[1],
                self.current_gps_coords[0], self.current_gps_coords[1]
            )
            return dist_m * 100.0

        # 3. Universal Calibrated Speed-Time Integration
        if self.drive_start_time is not None:
            elapsed_s = now - self.drive_start_time
            dist_m = elapsed_s * self.calibrated_speed_mps
            return dist_m * 100.0

        return 0.0

    def _set_state(self, new_state: str) -> None:
        """Updates internal motion state and publishes update topic."""
        if self.state != new_state:
            self.state = new_state
            status_msg = String()
            status_msg.data = new_state
            self.status_pub.publish(status_msg)
            self.get_logger().info(f"[STATE] -> {new_state}")

    def _publish_cmd_vel(self, vx: float, wz: float) -> None:
        """Sends velocity target to the mdd10_motor_controller."""
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

    def _publish_diagnostics(self) -> None:
        """Reads Raspberry Pi 5 SoC temperature directly from Linux sysfs with 0 overhead."""
        try:
            temp_c = None
            # Standard Raspberry Pi / Ubuntu 24.04 LTS thermal zone
            if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    raw_val = f.read().strip()
                    if raw_val:
                        temp_c = round(float(raw_val) / 1000.0, 1)

            if temp_c is not None:
                diag_msg = String()
                diag_msg.data = json.dumps({"cpu_temp": f"{temp_c:.1f} °C", "temp_val": temp_c})
                self.diag_pub.publish(diag_msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        """Clean stop on node shutdown."""
        self._publish_cmd_vel(0.0, 0.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
