#!/usr/bin/env python3
"""
test_action_motion.py - Safe CLI Runner for ConeRobot ExecuteMotion Action.

Problem this solves:
  Standard `ros2 action send_goal` from the command line does NOT cleanly
  cancel goals on Ctrl+C (it terminates the CLI tool while leaving the
  action server driving the robot).

This script:
  1. Sends the goal to /execute_motion.
  2. Prints real-time distance and heading feedback.
  3. Traps Ctrl+C (KeyboardInterrupt) and sends an immediate CANCEL request,
     ensuring the robot stops the exact instant you press Ctrl+C.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

try:
    from cone_robot_interfaces.action import ExecuteMotion
except ImportError:
    print("[ERROR] cone_robot_interfaces not found. Did you run: source ~/ros2_ws/install/setup.bash ?")
    sys.exit(1)


def gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute local flat-Earth metric distance between two WGS84 GPS coordinates."""
    earth_radius_m = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    x = d_lon * math.cos(lat_avg)
    y = d_lat
    return math.sqrt(x * x + y * y) * earth_radius_m


def gps_enu_delta_m(lat1: float, lon1: float, lat2: float, lon2: float):
    """Return (east_m, north_m) from (lat1, lon1) to (lat2, lon2)."""
    earth_radius_m = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    east_m = d_lon * math.cos(lat_avg) * earth_radius_m
    north_m = d_lat * earth_radius_m
    return east_m, north_m


class SafeActionRunner(Node):
    def __init__(self):
        super().__init__('safe_action_runner')
        self._action_client = ActionClient(self, ExecuteMotion, 'execute_motion')
        self._goal_handle = None
        self._is_done = False

        # Real-time GPS RTK Fix, Base Station (0,0), LiDAR /odom, Position & /cmd_vel Monitor
        self.gps_status_sub = self.create_subscription(String, '/gps/status', self._gps_status_cb, 10)
        self.gps_fix_sub = self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)
        self.base_fix_sub = self.create_subscription(NavSatFix, '/gps/base_fix', self._base_fix_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 20)
        self.current_fix_quality = "UNKNOWN"
        self.num_sats = 0
        self.hdop = 99.99
        self.initial_fix_quality = None
        self.drop_events = []
        self.motion_start_time = None

        self.current_pos = None
        self.base_pos = None
        self.start_pos = None
        self.current_odom_pos = None
        self.start_odom_pos = None
        self._last_odom_log_time = 0.0
        self.waypoint_target_gps = None
        self.waypoint_target_base_xy = None
        self.target_distance = 0.0
        self.active_motion_type = 0

        # Detailed 1 Hz Fix & Motor Cut-Off Telemetry Loggers
        self.fix_history = []
        self._prev_fix_time = None
        self._prev_fix_dist = 0.0
        self._cmd_v_sum = 0.0
        self._cmd_v_count = 0
        self.last_cmd_v = 0.0
        self.last_nonzero_cmd_v = 0.0
        self.motor_cut_events = []
        self._was_moving = False

    def _cmd_vel_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        now = time.time()
        self.last_cmd_v = v
        if self.motion_start_time is not None:
            self._cmd_v_sum += abs(v)
            self._cmd_v_count += 1
            is_moving = abs(v) > 0.005 or abs(w) > 0.01
            if is_moving:
                self.last_nonzero_cmd_v = v
                self._was_moving = True
            elif self._was_moving and not is_moving:
                self._was_moving = False
                elapsed = now - self.motion_start_time
                dt_since_fix = (now - self._prev_fix_time) if self._prev_fix_time else 0.0
                self.motor_cut_events.append({
                    "t": elapsed,
                    "last_fix_dist": self._prev_fix_dist,
                    "dt_since_fix": dt_since_fix,
                    "prev_cmd_v": self.last_nonzero_cmd_v,
                })

    def _base_fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.base_pos = (msg.latitude, msg.longitude)

    def _odom_cb(self, msg: Odometry):
        ox = float(msg.pose.pose.position.x)
        oy = float(msg.pose.pose.position.y)
        now = time.time()
        self.current_odom_pos = (ox, oy)
        if self.motion_start_time and self.start_odom_pos is None:
            self.start_odom_pos = (ox, oy)
            if self.current_pos is None:
                self._prev_fix_time = now
                self._prev_fix_dist = 0.0
                self._last_odom_log_time = now

        # On a LiDAR-only robot (no GPS /fix), record 1 Hz telemetry rows from LiDAR /odom
        if self.motion_start_time and self.current_pos is None and self.start_odom_pos is not None:
            if (now - self._last_odom_log_time) >= 0.95:
                elapsed = now - self.motion_start_time
                dist_m = math.hypot(ox - self.start_odom_pos[0], oy - self.start_odom_pos[1])
                dt_fix = (now - self._prev_fix_time) if self._prev_fix_time else 0.0
                step_m = dist_m - self._prev_fix_dist
                true_v = (step_m / dt_fix) if dt_fix > 0.1 else 0.0
                avg_cmd_v = (self._cmd_v_sum / self._cmd_v_count) if self._cmd_v_count > 0 else abs(self.last_cmd_v)
                scale = (true_v / avg_cmd_v) if avg_cmd_v > 0.015 else 0.0

                self.fix_history.append({
                    "idx": len(self.fix_history) + 1,
                    "t": elapsed,
                    "dt": dt_fix,
                    "dist_m": dist_m,
                    "step_cm": step_m * 100.0,
                    "true_v": true_v,
                    "avg_cmd_v": avg_cmd_v,
                    "curr_cmd_v": self.last_cmd_v,
                    "scale": scale,
                    "lat": ox,
                    "lon": oy,
                })
                self._prev_fix_time = now
                self._prev_fix_dist = dist_m
                self._last_odom_log_time = now
                self._cmd_v_sum = 0.0
                self._cmd_v_count = 0

    def _fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            now = time.time()
            self.current_pos = (msg.latitude, msg.longitude)
            if self.motion_start_time and self.start_pos is None:
                self.start_pos = (msg.latitude, msg.longitude)
                self._prev_fix_time = now
                self._prev_fix_dist = 0.0

            if self.motion_start_time and self.start_pos is not None:
                elapsed = now - self.motion_start_time
                dist_m = gps_distance_m(
                    self.start_pos[0], self.start_pos[1],
                    msg.latitude, msg.longitude,
                )
                dt_fix = (now - self._prev_fix_time) if self._prev_fix_time else 0.0
                step_m = dist_m - self._prev_fix_dist
                true_v = (step_m / dt_fix) if dt_fix > 0.1 else 0.0
                avg_cmd_v = (self._cmd_v_sum / self._cmd_v_count) if self._cmd_v_count > 0 else abs(self.last_cmd_v)
                scale = (true_v / avg_cmd_v) if avg_cmd_v > 0.015 else 0.0

                self.fix_history.append({
                    "idx": len(self.fix_history) + 1,
                    "t": elapsed,
                    "dt": dt_fix,
                    "dist_m": dist_m,
                    "step_cm": step_m * 100.0,
                    "true_v": true_v,
                    "avg_cmd_v": avg_cmd_v,
                    "curr_cmd_v": self.last_cmd_v,
                    "scale": scale,
                    "lat": msg.latitude,
                    "lon": msg.longitude,
                })
                self._prev_fix_time = now
                self._prev_fix_dist = dist_m
                self._cmd_v_sum = 0.0
                self._cmd_v_count = 0

    def _gps_status_cb(self, msg: String):
        text = msg.data
        if "Fix:" in text:
            parts = text.split('|')
            fix_part = parts[0].replace("Fix:", "").strip()
            prev_quality = self.current_fix_quality
            self.current_fix_quality = fix_part

            if self.initial_fix_quality is None and fix_part != "UNKNOWN":
                self.initial_fix_quality = fix_part

            for p in parts[1:]:
                p_strip = p.strip()
                if p_strip.startswith("Sats:"):
                    try:
                        self.num_sats = int(p_strip.split(":")[1].strip())
                    except ValueError:
                        pass
                elif p_strip.startswith("HDOP:"):
                    try:
                        self.hdop = float(p_strip.split(":")[1].strip())
                    except ValueError:
                        pass

            # Detect degradation (drop) events immediately during active motion
            if prev_quality != "UNKNOWN" and fix_part != prev_quality:
                elapsed = (time.time() - self.motion_start_time) if self.motion_start_time else 0.0
                if "FIX" in prev_quality and "FIX" not in fix_part:
                    event = f"⚠️ [GPS DROP] Degraded from {prev_quality} -> {fix_part} at t={elapsed:.1f}s (Sats: {self.num_sats}, HDOP: {self.hdop:.2f})"
                    self.drop_events.append(event)
                    sys.stdout.write(f"\n\n{event}\n\n")
                    sys.stdout.flush()
                elif "FLOAT" in prev_quality and "FIX" in fix_part:
                    sys.stdout.write(f"\n\n🟢 [GPS RECOVERED] Upgraded to {fix_part} at t={elapsed:.1f}s!\n\n")
                    sys.stdout.flush()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        self.get_logger().info("Connecting to /execute_motion Action Server...")
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, motion_type: int, distance: float, delta_yaw: float, max_v: float, max_w: float):
        self.motion_start_time = time.time()
        self.active_motion_type = motion_type
        self.target_distance = float(distance)
        self.start_pos = self.current_pos
        self.start_odom_pos = self.current_odom_pos
        self._last_odom_log_time = self.motion_start_time
        self.fix_history = []
        self.motor_cut_events = []
        self._prev_fix_time = self.motion_start_time
        self._prev_fix_dist = 0.0
        self._cmd_v_sum = 0.0
        self._cmd_v_count = 0
        self._was_moving = False

        goal_msg = ExecuteMotion.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.distance = float(distance)
        goal_msg.delta_yaw = float(delta_yaw)
        goal_msg.max_velocity = float(max_v)
        goal_msg.max_yaw_rate = float(max_w)

        self.get_logger().info(
            f"Sending Goal: Type={motion_type}, Dist={distance:.2f}m, Yaw={math.degrees(delta_yaw):.1f}°, MaxV={max_v:.2f}m/s"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self._goal_handle = send_goal_future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected by Action Server.")
            return False

        self.get_logger().info("✅ Goal accepted by robot! Executing motion...")
        return True

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        pct = fb.progress_ratio * 100.0

        # Real-time GPS or LiDAR Odometry status badge
        if "RTK FIX" in self.current_fix_quality:
            gps_badge = "🟢 RTK FIX"
        elif "RTK FLOAT" in self.current_fix_quality:
            gps_badge = "🟡 RTK FLOAT"
        elif "3D" in self.current_fix_quality:
            gps_badge = "🔵 3D FIX"
        elif self.current_odom_pos is not None:
            gps_badge = "🟣 LIDAR ODOM"
        else:
            gps_badge = f"⚪ {self.current_fix_quality}"

        sat_str = f"|{self.num_sats}s" if self.num_sats > 0 else ""
        sys.stdout.write(
            f"\rExecuting: [{pct:5.1f}%] | Dist Rem: {fb.distance_remaining:4.2f}m | Yaw Rem: {math.degrees(fb.yaw_remaining):5.1f}° | Speed: {fb.current_velocity:4.2f}m/s | Sensor: [{gps_badge}{sat_str}]   "
        )
        sys.stdout.flush()

    def wait_for_result(self):
        if not self._goal_handle:
            return
        result_future = self._goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        if result_future.done():
            res = result_future.result()
            print()
            if res.result.success:
                self.get_logger().info(
                    f"🏆 Motion Succeeded! Distance: {res.result.actual_distance:.2f}m, Yaw: {math.degrees(res.result.actual_yaw):.1f}°"
                )
            else:
                self.get_logger().warn(
                    f"⚠️ Motion Finished with Status: {res.result.message} (Code: {res.result.error_code})"
                )

            # Print Linear Displacement Accuracy Benchmark for Straight motions (forward & reverse)
            if self.start_pos and self.current_pos and self.active_motion_type == 0 and abs(self.target_distance) > 0.05:
                pos_at_completion = self.current_pos
                dist_at_completion = gps_distance_m(
                    self.start_pos[0], self.start_pos[1],
                    pos_at_completion[0], pos_at_completion[1],
                )

                # Spin for 1.1s after action completion to capture 1 extra stationary /fix
                # and verify whether any post-completion GPS lag or coasting occurred
                post_wait_end = time.time() + 1.15
                while rclpy.ok() and time.time() < post_wait_end:
                    rclpy.spin_once(self, timeout_sec=0.05)

                target_mag = abs(self.target_distance)
                actual_gps_dist = gps_distance_m(self.start_pos[0], self.start_pos[1], self.current_pos[0], self.current_pos[1])
                post_settle_drift_cm = (actual_gps_dist - dist_at_completion) * 100.0
                err_m = actual_gps_dist - target_mag
                err_cm = abs(err_m) * 100.0

                if self.fix_history:
                    print("\n" + "=" * 78)
                    print(" 📊 1 Hz RTK GPS STEP-BY-STEP TELEMETRY & MOTOR CUT-OFF LOG")
                    print("=" * 78)
                    print(f" {'#':>2} | {'t (s)':>6} | {'GPS Dist':>9} | {'Step':>8} | {'True V':>8} | {'AvgCmdV':>8} | {'NowCmdV':>8} | {'Scale':>5}")
                    print("-" * 78)
                    for row in self.fix_history:
                        scale_str = f"{row['scale']:.2f}x" if row['scale'] > 0 else "  -  "
                        print(
                            f" {row['idx']:2d} | {row['t']:5.2f}s | {row['dist_m']:8.3f}m | "
                            f"{row['step_cm']:+6.1f}cm | {row['true_v']:6.2f}m/s | "
                            f"{row['avg_cmd_v']:6.2f}m/s | {row['curr_cmd_v']:+6.2f}m/s | {scale_str}"
                        )
                    print("-" * 78)
                    if self.motor_cut_events:
                        for idx, ev in enumerate(self.motor_cut_events, 1):
                            print(
                                f" 🛑 Motor Cut #{idx} at t={ev['t']:.2f}s "
                                f"(+{ev['dt_since_fix']:.2f}s after Fix@{ev['last_fix_dist']:.3f}m, "
                                f"last CmdV={ev['prev_cmd_v']:+.3f}m/s)"
                            )
                    print("=" * 78)

                print("\n" + "=" * 65)
                print(" 🎯 LINEAR DISPLACEMENT ACCURACY BENCHMARK (RTK GPS)")
                print("=" * 65)
                print(f" Commanded Target Distance : {target_mag:.3f} m ({self.target_distance:+.2f} m)")
                print(f" Distance at Action Done   : {dist_at_completion:.3f} m")
                print(f" Measured Stationary (+1s) : {actual_gps_dist:.3f} m (Post-done drift: {post_settle_drift_cm:+.1f} cm)")
                print(f" Net Displacement Error    : {err_cm:.1f} cm ({err_m:+.3f} m)")
                print(f" Start Position (Lat, Lon) : ({self.start_pos[0]:.7f}, {self.start_pos[1]:.7f})")
                print(f" Final Position (Lat, Lon) : ({self.current_pos[0]:.7f}, {self.current_pos[1]:.7f})")
                print(f" Final Heading Error       : {math.degrees(res.result.actual_yaw):+.1f}°")
                if err_cm <= 2.5:
                    print(" 🏆 GRADE: SURVEY-GRADE (< 2.5 cm precision) - Flawless RTK performance!")
                elif err_cm <= 5.0:
                    print(" 🟢 GRADE: HIGH PRECISION (< 5.0 cm precision) - Excellent linear driving!")
                elif err_cm <= 15.0:
                    print(" 🟡 GRADE: MODERATE (~5-15 cm) - Minor track slip or RTK Float jitter.")
                else:
                    print(" ⚠️ GRADE: ELEVATED ERROR (> 15 cm) - Check RTK Fix & motor traction.")
                print("=" * 65)

            # Print Loop Closure Accuracy Benchmark for Circular Arc motions
            elif self.start_pos and self.current_pos and self.active_motion_type == 2:
                gap_m = gps_distance_m(self.start_pos[0], self.start_pos[1], self.current_pos[0], self.current_pos[1])
                gap_cm = gap_m * 100.0

                print("\n" + "=" * 65)
                print(" 🔄 CIRCULAR LOOP CLOSURE ACCURACY BENCHMARK (RTK GPS)")
                print("=" * 65)
                print(f" Path Target Distance      : {self.target_distance:.3f} m")
                print(f" Start Position (Lat, Lon) : ({self.start_pos[0]:.7f}, {self.start_pos[1]:.7f})")
                print(f" Final Position (Lat, Lon) : ({self.current_pos[0]:.7f}, {self.current_pos[1]:.7f})")
                print(f" Loop Closure Gap Distance : {gap_cm:.1f} cm ({gap_m:+.3f} m from start)")
                print(f" Final Heading Error       : {math.degrees(res.result.actual_yaw) % 360.0:.1f}° (relative to 360°)")
                if gap_cm <= 3.0:
                    print(" 🏆 GRADE: SURVEY-GRADE (< 3.0 cm gap) - Perfect closed loop!")
                elif gap_cm <= 8.0:
                    print(" 🟢 GRADE: HIGH PRECISION (< 8.0 cm gap) - Excellent circle closure!")
                elif gap_cm <= 20.0:
                    print(" 🟡 GRADE: MODERATE (~8-20 cm) - Skid-steer track scrub or RTK Float jitter.")
                else:
                    print(" ⚠️ GRADE: ELEVATED ERROR (> 20 cm) - Incomplete turn or track slip.")
                print("=" * 65)

            # Print Waypoint Navigation Accuracy Benchmark for GOTO_BASE_XY (4) & GOTO_REL_ENU (5)
            elif self.current_pos and self.active_motion_type in (4, 5):
                post_wait_end = time.time() + 1.15
                while rclpy.ok() and time.time() < post_wait_end:
                    rclpy.spin_once(self, timeout_sec=0.05)

                print("\n" + "=" * 65)
                print(" 🧭 WAYPOINT NAVIGATION ACCURACY BENCHMARK (RTK GPS)")
                print("=" * 65)
                if self.start_pos:
                    print(f" Start Position (Lat, Lon)  : ({self.start_pos[0]:.7f}, {self.start_pos[1]:.7f})")
                print(f" Final Position (Lat, Lon)  : ({self.current_pos[0]:.7f}, {self.current_pos[1]:.7f})")

                err_cm = None
                if self.active_motion_type == 4 and self.base_pos and self.waypoint_target_base_xy:
                    tx, ty = self.waypoint_target_base_xy
                    fx, fy = gps_enu_delta_m(
                        self.base_pos[0], self.base_pos[1],
                        self.current_pos[0], self.current_pos[1],
                    )
                    err_m = math.hypot(tx - fx, ty - fy)
                    err_cm = err_m * 100.0
                    print(f" RTK Base (0,0) Coordinate  : ({self.base_pos[0]:.7f}, {self.base_pos[1]:.7f})")
                    print(f" Target Base XY (E, N)      : ({tx:+.3f} m, {ty:+.3f} m)")
                    print(f" Final Base XY  (E, N)      : ({fx:+.3f} m, {fy:+.3f} m)")
                    print(f" 2D Arrival Error           : {err_cm:.1f} cm (dX={fx - tx:+.3f}m, dY={fy - ty:+.3f}m)")
                elif self.waypoint_target_gps:
                    tlat, tlon = self.waypoint_target_gps
                    de, dn = gps_enu_delta_m(
                        tlat, tlon,
                        self.current_pos[0], self.current_pos[1],
                    )
                    err_m = math.hypot(de, dn)
                    err_cm = err_m * 100.0
                    print(f" Target Global (Lat, Lon)   : ({tlat:.7f}, {tlon:.7f})")
                    if self.base_pos:
                        fx, fy = gps_enu_delta_m(
                            self.base_pos[0], self.base_pos[1],
                            self.current_pos[0], self.current_pos[1],
                        )
                        print(f" Final Base XY  (E, N)      : ({fx:+.3f} m, {fy:+.3f} m)")
                    print(f" 2D Arrival Error           : {err_cm:.1f} cm (dE={de:+.3f}m, dN={dn:+.3f}m)")

                if err_cm is not None:
                    if err_cm <= 3.0:
                        print(" 🏆 GRADE: SURVEY-GRADE (< 3.0 cm radius) - Pinpoint waypoint arrival!")
                    elif err_cm <= 6.0:
                        print(" 🟢 GRADE: HIGH PRECISION (< 6.0 cm radius) - Accurate waypoint arrival!")
                    elif err_cm <= 15.0:
                        print(" 🟡 GRADE: MODERATE (~6-15 cm) - Minor final alignment/slip offset.")
                    else:
                        print(" ⚠️ GRADE: ELEVATED ERROR (> 15 cm) - Check RTK Fix & compass/COG alignment.")
                print("=" * 65)

            # LiDAR-Only Robot Fallback Benchmarks (when /fix is absent and /odom is active)
            elif self.current_pos is None and self.start_odom_pos and self.current_odom_pos:
                sx, sy = self.start_odom_pos
                fx, fy = self.current_odom_pos
                odom_dist = math.hypot(fx - sx, fy - sy)
                if self.fix_history and self.active_motion_type == 0:
                    print("\n" + "=" * 78)
                    print(" 📊 1 Hz LIDAR ODOMETRY STEP-BY-STEP TELEMETRY & MOTOR CUT-OFF LOG")
                    print("=" * 78)
                    print(f" {'#':>2} | {'t (s)':>6} | {'OdomDist':>9} | {'Step':>8} | {'True V':>8} | {'AvgCmdV':>8} | {'NowCmdV':>8} | {'Scale':>5}")
                    print("-" * 78)
                    for row in self.fix_history:
                        scale_str = f"{row['scale']:.2f}x" if row['scale'] > 0 else "  -  "
                        print(
                            f" {row['idx']:2d} | {row['t']:5.2f}s | {row['dist_m']:8.3f}m | "
                            f"{row['step_cm']:+6.1f}cm | {row['true_v']:6.2f}m/s | "
                            f"{row['avg_cmd_v']:6.2f}m/s | {row['curr_cmd_v']:+6.2f}m/s | {scale_str}"
                        )
                    print("=" * 78)

                if self.active_motion_type == 0 and abs(self.target_distance) > 0.05:
                    target_mag = abs(self.target_distance)
                    err_m = odom_dist - target_mag
                    err_cm = abs(err_m) * 100.0
                    print("\n" + "=" * 65)
                    print(" 🎯 LINEAR DISPLACEMENT ACCURACY BENCHMARK (LIDAR ODOMETRY)")
                    print("=" * 65)
                    print(f" Commanded Target Distance : {target_mag:.3f} m ({self.target_distance:+.2f} m)")
                    print(f" Measured LiDAR Odom Dist  : {odom_dist:.3f} m")
                    print(f" Net Displacement Error    : {err_cm:.1f} cm ({err_m:+.3f} m)")
                    print(f" Start Odom (X, Y)         : ({sx:+.3f} m, {sy:+.3f} m)")
                    print(f" Final Odom (X, Y)         : ({fx:+.3f} m, {fy:+.3f} m)")
                    print(f" Final Heading Error       : {math.degrees(res.result.actual_yaw):+.1f}°")
                    print("=" * 65)
                elif self.active_motion_type == 2:
                    gap_cm = odom_dist * 100.0
                    print("\n" + "=" * 65)
                    print(" 🔄 CIRCULAR LOOP CLOSURE ACCURACY BENCHMARK (LIDAR ODOMETRY)")
                    print("=" * 65)
                    print(f" Path Target Distance      : {self.target_distance:.3f} m")
                    print(f" Start Odom (X, Y)         : ({sx:+.3f} m, {sy:+.3f} m)")
                    print(f" Final Odom (X, Y)         : ({fx:+.3f} m, {fy:+.3f} m)")
                    print(f" Loop Closure Gap Distance : {gap_cm:.1f} cm ({odom_dist:+.3f} m from start)")
                    print("=" * 65)
                elif self.active_motion_type in (4, 5) and self.waypoint_target_base_xy:
                    tx, ty = self.waypoint_target_base_xy
                    err_cm = math.hypot(fx - tx, fy - ty) * 100.0
                    print("\n" + "=" * 65)
                    print(" 🧭 WAYPOINT NAVIGATION ACCURACY BENCHMARK (LIDAR ODOMETRY)")
                    print("=" * 65)
                    print(f" Target Odom XY            : ({tx:+.3f} m, {ty:+.3f} m)")
                    print(f" Final  Odom XY            : ({fx:+.3f} m, {fy:+.3f} m)")
                    print(f" 2D Arrival Error          : {err_cm:.1f} cm (dX={fx - tx:+.3f}m, dY={fy - ty:+.3f}m)")
                    print("=" * 65)

            # Print GPS / Sensor Performance & Stability Report
            if self.current_pos is not None or self.initial_fix_quality is not None:
                print("\n" + "=" * 65)
                print(" 📡 RTK GPS INTEGRITY & STABILITY REPORT")
                print("=" * 65)
                print(f" Initial Status at Start : {self.initial_fix_quality or 'N/A'}")
                print(f" Final Status at Finish  : {self.current_fix_quality} (Sats: {self.num_sats}, HDOP: {self.hdop:.2f})")
                if not self.drop_events:
                    if "RTK FIX" in self.current_fix_quality:
                        print(" 🏆 PERFECT RTK LOCK: Fix remained solid RTK FIX throughout entire motion!")
                    else:
                        print(f" ℹ️ Constant State: Remained in {self.current_fix_quality} (no state drop observed).")
                else:
                    print(f" ⚠️ {len(self.drop_events)} DROP EVENT(S) DETECTED DURING MOTION:")
                    for ev in self.drop_events:
                        print(f"    • {ev}")
                print("=" * 65 + "\n")
            elif self.current_odom_pos is not None:
                print(f"\n🟣 [SENSOR REPORT] Motion executed purely on LiDAR Odometry (/odom).\n")

    def cancel_active_goal(self):
        """Immediately sends cancel request and halts robot."""
        if self._goal_handle and self._goal_handle.status in [1, 2]: # ACCEPTED or EXECUTING
            print("\n🛑 Ctrl+C detected! Sending CANCEL request to robot...")
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            print("✅ Robot safely halted.")
        else:
            print("\n🛑 No active goal to cancel.")


def main():
    parser = argparse.ArgumentParser(description="Safe Action Motion Runner for ConeRobot")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Circle Arc subparser
    arc_parser = subparsers.add_parser("circle", help="Drive a circular arc")
    arc_parser.add_argument("--radius", type=float, default=1.0, help="Circle radius in meters (default: 1.0 m)")
    arc_parser.add_argument("--speed", type=float, default=0.20, help="Linear speed in m/s (default: 0.20 m/s)")
    arc_parser.add_argument("--direction", choices=["left", "right"], default="left", help="Turn direction (default: left / CCW)")

    # Straight subparser
    straight_parser = subparsers.add_parser("straight", help="Drive straight")
    straight_parser.add_argument("--distance", type=float, default=1.0, help="Distance in meters (default: 1.0 m)")
    straight_parser.add_argument("--speed", type=float, default=0.25, help="Speed in m/s (default: 0.25 m/s)")

    # Rotate subparser
    rotate_parser = subparsers.add_parser("rotate", help="Rotate in place")
    rotate_parser.add_argument("--deg", type=float, default=90.0, help="Angle in degrees (+ left, - right)")
    rotate_parser.add_argument("--rate", type=float, default=1.0, help="Yaw rate in rad/s")

    # Waypoint GoTo subparser (supports BOTH Base-relative X/Y meters AND Global WGS84 Lat/Lon)
    goto_parser = subparsers.add_parser(
        "goto",
        help="Navigate to a 2D waypoint using Base-Station (0,0) meters (--x --y) OR Global GPS (--lat --lon)",
    )
    goto_parser.add_argument("--x", type=float, default=None, help="Target East (+X) in meters relative to RTK Base Station (0,0)")
    goto_parser.add_argument("--y", type=float, default=None, help="Target North (+Y) in meters relative to RTK Base Station (0,0)")
    goto_parser.add_argument("--lat", type=float, default=None, help="Target Global WGS84 Latitude (decimal degrees)")
    goto_parser.add_argument("--lon", type=float, default=None, help="Target Global WGS84 Longitude (decimal degrees)")
    goto_parser.add_argument("--speed", type=float, default=0.25, help="Max linear speed in m/s (default: 0.25 m/s)")
    goto_parser.add_argument("--rate", type=float, default=0.8, help="Max turn yaw rate in rad/s (default: 0.8 rad/s)")

    # Stop subparser
    subparsers.add_parser("stop", help="Emergency stop active motion")

    args = parser.parse_args()

    rclpy.init()
    runner = SafeActionRunner()

    if not runner.wait_for_server(timeout_sec=5.0):
        print("❌ Action server /execute_motion not available. Is primitive_motion_controller running?")
        runner.destroy_node()
        rclpy.shutdown()
        return

    try:
        # Spin briefly to capture initial /fix or /odom before recording start_pos / start_odom_pos
        init_wait = time.time() + 0.6
        while rclpy.ok() and time.time() < init_wait:
            rclpy.spin_once(runner, timeout_sec=0.05)
            if runner.current_pos is not None or runner.current_odom_pos is not None:
                break

        if args.command == "circle":
            r = abs(args.radius)
            circumference = 2.0 * math.pi * r
            sign = 1.0 if args.direction == "left" else -1.0
            delta_yaw = sign * 2.0 * math.pi
            w_max = args.speed / r
            if runner.send_goal(ExecuteMotion.Goal.TYPE_DRIVE_ARC, circumference, delta_yaw, args.speed, w_max):
                runner.wait_for_result()

        elif args.command == "straight":
            if runner.send_goal(ExecuteMotion.Goal.TYPE_DRIVE_STRAIGHT, args.distance, 0.0, args.speed, 0.0):
                runner.wait_for_result()

        elif args.command == "rotate":
            yaw_rad = math.radians(args.deg)
            if runner.send_goal(ExecuteMotion.Goal.TYPE_ROTATE_IN_PLACE, 0.0, yaw_rad, 0.0, args.rate):
                runner.wait_for_result()

        elif args.command == "goto":
            has_xy = (args.x is not None) and (args.y is not None)
            has_latlon = (args.lat is not None) and (args.lon is not None)
            if not has_xy and not has_latlon:
                print("❌ Error: For 'goto', provide either --x <east_m> --y <north_m> (Base/Odom 0,0) OR --lat <deg> --lon <deg> (Global).")
                return

            # Spin briefly to receive current /fix, /gps/base_fix, or /odom
            wait_until = time.time() + 2.0
            while rclpy.ok() and time.time() < wait_until:
                rclpy.spin_once(runner, timeout_sec=0.05)
                if runner.current_pos is not None and (has_latlon or runner.base_pos is not None):
                    break
                if runner.current_pos is None and runner.current_odom_pos is not None and has_xy:
                    break

            if has_xy:
                runner.waypoint_target_base_xy = (args.x, args.y)
                print(
                    f"🧭 Navigating to Base-Station (0,0) Waypoint: "
                    f"X_East={args.x:+.2f}m, Y_North={args.y:+.2f}m"
                )
                m_type = getattr(ExecuteMotion.Goal, 'TYPE_GOTO_BASE_XY', 4)
                if runner.send_goal(m_type, args.x, args.y, args.speed, args.rate):
                    runner.wait_for_result()
            else:
                if runner.current_pos is None:
                    print("❌ Error: No /fix received from GPS node yet; cannot compute global waypoint vector.")
                    return
                runner.waypoint_target_gps = (args.lat, args.lon)
                de_m, dn_m = gps_enu_delta_m(
                    runner.current_pos[0], runner.current_pos[1],
                    args.lat, args.lon,
                )
                print(
                    f"🌍 Navigating to Global WGS84 Waypoint: ({args.lat:.7f}, {args.lon:.7f}) "
                    f"[Offset: dEast={de_m:+.2f}m, dNorth={dn_m:+.2f}m]"
                )
                m_type = getattr(ExecuteMotion.Goal, 'TYPE_GOTO_REL_ENU', 5)
                if runner.send_goal(m_type, de_m, dn_m, args.speed, args.rate):
                    runner.wait_for_result()

        elif args.command == "stop":
            print("Sending TYPE_STOP...")
            runner.send_goal(ExecuteMotion.Goal.TYPE_STOP, 0.0, 0.0, 0.0, 0.0)
            runner.wait_for_result()

    except KeyboardInterrupt:
        runner.cancel_active_goal()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
