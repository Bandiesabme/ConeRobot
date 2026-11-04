#!/usr/bin/env python3
"""
test_waypoint_navigation.py - Comprehensive Test & Diagnostic Suite for:
  1. 25-Sample Startup Compass Calibration + 6-DOF Gyro Stability
  2. Automatic RTCM 1005/1006 Base Station (0, 0) Coordinate Decoding
  3. Waypoint Navigation in Base-Station (0, 0) Meters (--x --y)
  4. Waypoint Navigation in Global WGS84 Coordinates (--lat --lon)
  5. Relative East/North Offset Test (--east --north) with optional --return to start

Safety:
  Traps Ctrl+C (KeyboardInterrupt) and sends an immediate CANCEL request to
  /execute_motion so the robot halts immediately.
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
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

try:
    from cone_robot_interfaces.action import ExecuteMotion
except ImportError:
    print("[ERROR] cone_robot_interfaces not found. Did you run: source install/setup.bash ?")
    sys.exit(1)


def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def shortest_diff_deg(target_deg: float, source_deg: float) -> float:
    return normalize_angle_deg(target_deg - source_deg)


def gps_enu_delta_m(lat1: float, lon1: float, lat2: float, lon2: float):
    """Return (east_m, north_m) from (lat1, lon1) to (lat2, lon2)."""
    earth_radius_m = 6371000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    east_m = d_lon * math.cos(lat_avg) * earth_radius_m
    north_m = d_lat * earth_radius_m
    return east_m, north_m


def gps_enu_to_latlon(lat_ref: float, lon_ref: float, east_m: float, north_m: float):
    """Convert local (east_m, north_m) offset from (lat_ref, lon_ref) to WGS84 (lat, lon)."""
    earth_radius_m = 6371000.0
    d_lat = math.degrees(north_m / earth_radius_m)
    cos_lat = max(1e-6, math.cos(math.radians(lat_ref)))
    d_lon = math.degrees(east_m / (earth_radius_m * cos_lat))
    return lat_ref + d_lat, lon_ref + d_lon


class WaypointTestRunner(Node):
    def __init__(self):
        super().__init__("waypoint_test_runner")
        self._action_client = ActionClient(self, ExecuteMotion, "execute_motion")
        self._goal_handle = None

        # Subscriptions
        self.fix_sub = self.create_subscription(NavSatFix, "/fix", self._fix_cb, 10)
        self.base_sub = self.create_subscription(NavSatFix, "/gps/base_fix", self._base_fix_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.status_sub = self.create_subscription(String, "/gps/status", self._gps_status_cb, 10)
        self.heading_sub = self.create_subscription(Float32, "/imu/heading", self._heading_cb, 20)
        self.compass_sub = self.create_subscription(Float32, "/imu/compass_heading", self._compass_cb, 20)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 20)

        # Live Sensor State
        self.current_pos = None          # (lat, lon)
        self.base_pos = None             # (lat, lon) of RTK Base Station (0, 0)
        self.current_odom_pos = None     # (x, y) from LiDAR /odom
        self.start_odom_pos = None
        self._last_odom_log_time = 0.0
        self.fix_quality = "UNKNOWN"
        self.num_sats = 0
        self.hdop = 99.99

        # IMU & Compass State
        self.imu_yaw_deg = None          # 6-DOF Game Rotation Vector (drift-free gyro)
        self.compass_yaw_deg = None      # 9-DOF Magnetometer-referenced compass
        self.compass_samples = []
        self.startup_compass_offset_deg = None
        self.rtk_cog_heading_deg = None
        self.rtk_cog_locked = False

        # Motion Telemetry
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.motion_start_time = None
        self.start_pos = None
        self.drive_anchor_pos = None
        self.drive_anchor_imu_deg = None
        self.step_logs = []

    def _cmd_vel_cb(self, msg: Twist):
        self.last_cmd_v = float(msg.linear.x)
        self.last_cmd_w = float(msg.angular.z)

    def _heading_cb(self, msg: Float32):
        self.imu_yaw_deg = normalize_angle_deg(float(msg.data))

    def _compass_cb(self, msg: Float32):
        comp_deg = normalize_angle_deg(float(msg.data))
        self.compass_yaw_deg = comp_deg
        if self.imu_yaw_deg is not None and len(self.compass_samples) < 30:
            off_rad = math.radians(shortest_diff_deg(comp_deg, self.imu_yaw_deg))
            self.compass_samples.append(off_rad)
            # Skip first 5 warmup samples when computing circular average
            valid = self.compass_samples[5:] if len(self.compass_samples) > 5 else self.compass_samples
            s_sin = sum(math.sin(a) for a in valid)
            s_cos = sum(math.cos(a) for a in valid)
            self.startup_compass_offset_deg = math.degrees(math.atan2(s_sin, s_cos))

    def _base_fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            if abs(msg.latitude) > 1e-4 or abs(msg.longitude) > 1e-4:
                self.base_pos = (float(msg.latitude), float(msg.longitude))

    def _odom_cb(self, msg: Odometry):
        ox = float(msg.pose.pose.position.x)
        oy = float(msg.pose.pose.position.y)
        self.current_odom_pos = (ox, oy)
        if self.motion_start_time is not None and self.current_pos is None:
            now = time.time()
            if self.start_odom_pos is None:
                self.start_odom_pos = (ox, oy)
                self._last_odom_log_time = now
            if (now - self._last_odom_log_time) >= 0.95:
                self._last_odom_log_time = now
                self.step_logs.append({
                    "idx": len(self.step_logs) + 1,
                    "t": now - self.motion_start_time,
                    "lat": ox,
                    "lon": oy,
                    "base_x": ox,
                    "base_y": oy,
                    "cmd_v": self.last_cmd_v,
                    "cmd_w": self.last_cmd_w,
                    "compass_enu_deg": self.imu_yaw_deg,
                    "rtk_cog_deg": None,
                })

    def _gps_status_cb(self, msg: String):
        text = msg.data
        if "Fix:" in text:
            parts = text.split("|")
            self.fix_quality = parts[0].replace("Fix:", "").strip()
            for p in parts[1:]:
                ps = p.strip()
                if ps.startswith("Sats:"):
                    try:
                        self.num_sats = int(ps.split(":")[1].strip())
                    except ValueError:
                        pass
                elif ps.startswith("HDOP:"):
                    try:
                        self.hdop = float(ps.split(":")[1].strip())
                    except ValueError:
                        pass

    def _fix_cb(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        curr = (float(msg.latitude), float(msg.longitude))
        self.current_pos = curr

        if self.motion_start_time is not None:
            now = time.time()
            elapsed = now - self.motion_start_time
            if self.start_pos is None:
                self.start_pos = curr

            # Track when straight driving begins so we can observe RTK Course-Over-Ground (COG)
            if abs(self.last_cmd_v) > 0.03 and abs(self.last_cmd_w) < 0.15:
                if self.drive_anchor_pos is None:
                    self.drive_anchor_pos = curr
                    self.drive_anchor_imu_deg = self.imu_yaw_deg
                else:
                    de, dn = gps_enu_delta_m(
                        self.drive_anchor_pos[0], self.drive_anchor_pos[1],
                        curr[0], curr[1],
                    )
                    step_dist = math.hypot(de, dn)
                    if step_dist >= 0.20 and "RTK FIX" in self.fix_quality:
                        cog_deg = math.degrees(math.atan2(dn, de))
                        if self.last_cmd_v < 0:
                            cog_deg = normalize_angle_deg(cog_deg + 180.0)
                        self.rtk_cog_heading_deg = cog_deg
                        self.rtk_cog_locked = True

            # Record 1 Hz waypoint progress row
            base_x, base_y = (None, None)
            if self.base_pos is not None:
                base_x, base_y = gps_enu_delta_m(
                    self.base_pos[0], self.base_pos[1],
                    curr[0], curr[1],
                )
            est_enu_deg = None
            if self.imu_yaw_deg is not None and self.startup_compass_offset_deg is not None:
                est_enu_deg = normalize_angle_deg(self.imu_yaw_deg + self.startup_compass_offset_deg)

            self.step_logs.append({
                "idx": len(self.step_logs) + 1,
                "t": elapsed,
                "lat": curr[0],
                "lon": curr[1],
                "base_x": base_x,
                "base_y": base_y,
                "cmd_v": self.last_cmd_v,
                "cmd_w": self.last_cmd_w,
                "compass_enu_deg": est_enu_deg,
                "rtk_cog_deg": self.rtk_cog_heading_deg,
            })

    def wait_for_sensors(self, timeout_sec: float = 3.5):
        """Spin until we have GPS /fix (or LiDAR /odom) and 25 compass samples."""
        print("⏳ Collecting startup sensor data (RTK GPS / LiDAR Odom, Base Station ARP, 25-sample Compass)...")
        end_t = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)
            has_pos = (self.current_pos is not None and self.base_pos is not None) or (
                self.current_pos is None and self.current_odom_pos is not None
            )
            if has_pos and len(self.compass_samples) >= 30:
                break

    def print_sensor_diagnostic(self):
        print("\n" + "=" * 74)
        print(" 🧭 STARTUP HEADING & RTK BASE / LIDAR ODOM (0,0) DIAGNOSTIC REPORT")
        print("=" * 74)
        if self.current_pos:
            print(f" RTK GPS Status            : {self.fix_quality} (Sats: {self.num_sats}, HDOP: {self.hdop:.2f})")
            print(f" Robot Global (Lat, Lon)   : ({self.current_pos[0]:.7f}, {self.current_pos[1]:.7f})")
        elif self.current_odom_pos:
            print(" Active Position Sensor    : 🟣 LiDAR Odometry (/odom) [No GPS /fix detected]")
            print(f" Robot LiDAR Odom (X, Y)   : X = {self.current_odom_pos[0]:+.3f} m, Y = {self.current_odom_pos[1]:+.3f} m")
        else:
            print(f" RTK GPS Status            : {self.fix_quality} (Waiting for /fix or /odom...)")

        if self.base_pos:
            print(f" RTK Base (0,0) Coordinate : ({self.base_pos[0]:.7f}, {self.base_pos[1]:.7f}) [RTCM 1005/1006]")
            if self.current_pos:
                bx, by = gps_enu_delta_m(
                    self.base_pos[0], self.base_pos[1],
                    self.current_pos[0], self.current_pos[1],
                )
                dist_base = math.hypot(bx, by)
                print(f" Robot Pos Relative to Base: X_East = {bx:+.3f} m, Y_North = {by:+.3f} m (Dist: {dist_base:.2f} m)")
        elif self.current_pos is not None:
            print(" RTK Base (0,0) Coordinate : ⚠️ Not yet received on /gps/base_fix (wait ~5-10s for RTCM 1005)")

        if self.imu_yaw_deg is not None:
            print(f" 6-DOF Gyro Yaw (/imu/head): {self.imu_yaw_deg:+.1f}° (drift-free relative frame)")
        if self.compass_yaw_deg is not None and self.startup_compass_offset_deg is not None:
            enu_deg = normalize_angle_deg(self.imu_yaw_deg + self.startup_compass_offset_deg)
            n_used = max(1, len(self.compass_samples) - 5) if len(self.compass_samples) > 5 else len(self.compass_samples)
            print(f" Startup Compass Avg ({n_used:2d}x) : {enu_deg:+.1f}° ENU (0°=East, +90°=North, ±180°=West, -90°=South)")
        else:
            print(" Startup Compass Heading   : ⚠️ Waiting for /imu/compass_heading...")
        print("=" * 74)

    def execute_waypoint(
        self,
        motion_type: int,
        arg1: float,
        arg2: float,
        speed: float,
        rate: float,
        target_gps,
        target_base_xy=None,
        label: str = "WAYPOINT",
    ) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("❌ Action server /execute_motion not available!")
            return False

        self.motion_start_time = time.time()
        self.start_pos = self.current_pos
        self.start_odom_pos = self.current_odom_pos
        self._last_odom_log_time = self.motion_start_time
        self.drive_anchor_pos = None
        self.drive_anchor_imu_deg = None
        self.rtk_cog_heading_deg = None
        self.rtk_cog_locked = False
        self.step_logs = []

        goal_msg = ExecuteMotion.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.distance = float(arg1)
        goal_msg.delta_yaw = float(arg2)
        goal_msg.max_velocity = float(speed)
        goal_msg.max_yaw_rate = float(rate)

        print(f"\n🚀 [{label}] Sending Goal to /execute_motion (Type={motion_type}, V={speed:.2f}m/s, W={rate:.2f}rad/s)...")
        send_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if not self._goal_handle.accepted:
            print("❌ Goal was rejected by /execute_motion Action Server!")
            return False

        res_future = self._goal_handle.get_result_async()
        while rclpy.ok() and not res_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        # Wait 1.1s after completion to capture settled stationary RTK fix
        post_end = time.time() + 1.15
        while rclpy.ok() and time.time() < post_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        print()
        res = res_future.result().result
        self._print_waypoint_report(label, res, target_gps, target_base_xy)
        return bool(res.success)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        pct = fb.progress_ratio * 100.0
        phase = "ALIGN" if abs(self.last_cmd_v) < 0.01 and abs(self.last_cmd_w) > 0.02 else "DRIVE"
        cog_tag = f"COG:{self.rtk_cog_heading_deg:+.0f}°" if self.rtk_cog_heading_deg is not None else "Compass"
        sys.stdout.write(
            f"\r[{phase:5s} {pct:5.1f}%] Rem: {fb.distance_remaining:4.2f}m | "
            f"YawRem: {math.degrees(fb.yaw_remaining):+5.1f}° | "
            f"V: {fb.current_velocity:4.2f}m/s | HdgSrc: {cog_tag} | {self.fix_quality}   "
        )
        sys.stdout.flush()

    def _print_waypoint_report(self, label: str, res, target_gps, target_base_xy):
        if self.step_logs:
            print("\n" + "=" * 82)
            print(f" 📊 1 Hz WAYPOINT TELEMETRY LOG ({label})")
            print("=" * 82)
            print(f" {'#':>2} | {'t (s)':>6} | {'Base X (E)':>10} | {'Base Y (N)':>10} | {'Cmd V':>7} | {'Cmd W':>7} | {'Compass':>8} | {'RTK COG':>8}")
            print("-" * 82)
            for row in self.step_logs:
                bx_s = f"{row['base_x']:+8.3f}m" if row["base_x"] is not None else "    N/A   "
                by_s = f"{row['base_y']:+8.3f}m" if row["base_y"] is not None else "    N/A   "
                cp_s = f"{row['compass_enu_deg']:+6.1f}°" if row["compass_enu_deg"] is not None else "   -   "
                cg_s = f"{row['rtk_cog_deg']:+6.1f}°" if row["rtk_cog_deg"] is not None else "   -   "
                print(
                    f" {row['idx']:2d} | {row['t']:5.2f}s | {bx_s:>10} | {by_s:>10} | "
                    f"{row['cmd_v']:+5.2f}m/s | {row['cmd_w']:+5.2f}r/s | {cp_s:>8} | {cg_s:>8}"
                )
            print("=" * 82)

        print("\n" + "=" * 74)
        print(f" 🎯 WAYPOINT ARRIVAL BENCHMARK ({label})")
        print("=" * 74)
        print(f" Action Result             : {res.message} (Success: {res.success})")
        if self.start_pos and self.current_pos:
            traveled_e, traveled_n = gps_enu_delta_m(
                self.start_pos[0], self.start_pos[1],
                self.current_pos[0], self.current_pos[1],
            )
            traveled_m = math.hypot(traveled_e, traveled_n)
            true_course_deg = math.degrees(math.atan2(traveled_n, traveled_e))
            print(f" Start Global (Lat, Lon)   : ({self.start_pos[0]:.7f}, {self.start_pos[1]:.7f})")
            print(f" Final Global (Lat, Lon)   : ({self.current_pos[0]:.7f}, {self.current_pos[1]:.7f})")
            print(f" Net Traveled Vector (E,N) : ({traveled_e:+.3f} m, {traveled_n:+.3f} m) | Dist: {traveled_m:.3f} m | Course: {true_course_deg:+.1f}° ENU")

        if target_base_xy is not None and self.base_pos is not None and self.current_pos is not None:
            tx, ty = target_base_xy
            fx, fy = gps_enu_delta_m(
                self.base_pos[0], self.base_pos[1],
                self.current_pos[0], self.current_pos[1],
            )
            err_x = fx - tx
            err_y = fy - ty
            err_cm = math.hypot(err_x, err_y) * 100.0
            print(f" Target Base (0,0) XY      : X_East = {tx:+.3f} m, Y_North = {ty:+.3f} m")
            print(f" Final  Base (0,0) XY      : X_East = {fx:+.3f} m, Y_North = {fy:+.3f} m")
            print(f" 2D Arrival Error          : {err_cm:.1f} cm (dX_East = {err_x*100:+.1f} cm, dY_North = {err_y*100:+.1f} cm)")
        elif target_gps is not None and self.current_pos is not None:
            tlat, tlon = target_gps
            err_e, err_n = gps_enu_delta_m(tlat, tlon, self.current_pos[0], self.current_pos[1])
            err_cm = math.hypot(err_e, err_n) * 100.0
            print(f" Target Global (Lat, Lon)  : ({tlat:.7f}, {tlon:.7f})")
            print(f" 2D Arrival Error          : {err_cm:.1f} cm (dEast = {err_e*100:+.1f} cm, dNorth = {err_n*100:+.1f} cm)")
        elif self.current_pos is None and self.start_odom_pos is not None and self.current_odom_pos is not None:
            sx, sy = self.start_odom_pos
            fx, fy = self.current_odom_pos
            print(f" Start LiDAR Odom (X, Y)   : ({sx:+.3f} m, {sy:+.3f} m)")
            print(f" Final LiDAR Odom (X, Y)   : ({fx:+.3f} m, {fy:+.3f} m)")
            if target_base_xy is not None:
                tx, ty = target_base_xy
                err_x = fx - tx
                err_y = fy - ty
                err_cm = math.hypot(err_x, err_y) * 100.0
                print(f" Target LiDAR Odom (X, Y)  : ({tx:+.3f} m, {ty:+.3f} m)")
                print(f" 2D Arrival Error          : {err_cm:.1f} cm (dX = {err_x*100:+.1f} cm, dY = {err_y*100:+.1f} cm)")
        print("=" * 74 + "\n")

    def cancel_active_goal(self):
        if self._goal_handle and self._goal_handle.status in [1, 2]:
            print("\n🛑 Ctrl+C detected! Sending immediate CANCEL request...")
            fut = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            print("✅ Robot safely halted.")

    def pause_spin(self, duration_sec: float):
        """Non-blocking pause that keeps spinning ROS 2 callbacks to keep sensor states fresh."""
        end_t = time.time() + duration_sec
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description="ConeRobot Waypoint & Compass/RTK-Base Test Suite")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    # 1. Status / Diagnostic check (no movement)
    subparsers.add_parser(
        "status",
        help="Check 25-sample startup compass, RTK Base (0,0) coordinate, and robot's current Base XY & Global Lat/Lon",
    )

    # 2. Relative offset test from current spot (easiest field test!)
    off_p = subparsers.add_parser(
        "offset",
        help="Drive (dEast, dNorth) meters from current position using Base (0,0) or Global mode",
    )
    off_p.add_argument("--east", type=float, default=2.0, help="Meters East (+E / -W) from current spot (default: 2.0)")
    off_p.add_argument("--north", type=float, default=0.0, help="Meters North (+N / -S) from current spot (default: 0.0)")
    off_p.add_argument("--frame", choices=["base", "global"], default="base", help="Coordinate mode to test (default: base)")
    off_p.add_argument("--speed", type=float, default=0.25, help="Max linear speed in m/s (default: 0.25)")
    off_p.add_argument("--rate", type=float, default=0.8, help="Max turn rate in rad/s (default: 0.8)")
    off_p.add_argument("--return-start", action="store_true", help="Automatically return to exact starting coordinate after reaching target")

    # 3. Direct Base Station (0,0) XY coordinate
    base_p = subparsers.add_parser("base", help="Navigate to exact (X_East, Y_North) meters relative to RTK Base Station (0,0)")
    base_p.add_argument("--x", type=float, required=True, help="Target East (+X) in meters from RTK Base (0,0)")
    base_p.add_argument("--y", type=float, required=True, help="Target North (+Y) in meters from RTK Base (0,0)")
    base_p.add_argument("--speed", type=float, default=0.25, help="Max linear speed in m/s (default: 0.25)")
    base_p.add_argument("--rate", type=float, default=0.8, help="Max turn rate in rad/s (default: 0.8)")

    # 4. Direct Global WGS84 (Lat, Lon) coordinate
    glob_p = subparsers.add_parser("global", help="Navigate to exact Global WGS84 (Lat, Lon) coordinate")
    glob_p.add_argument("--lat", type=float, required=True, help="Target WGS84 Latitude (decimal degrees)")
    glob_p.add_argument("--lon", type=float, required=True, help="Target WGS84 Longitude (decimal degrees)")
    glob_p.add_argument("--speed", type=float, default=0.25, help="Max linear speed in m/s (default: 0.25)")
    glob_p.add_argument("--rate", type=float, default=0.8, help="Max turn rate in rad/s (default: 0.8)")

    args = parser.parse_args()

    rclpy.init()
    runner = WaypointTestRunner()

    try:
        runner.wait_for_sensors(timeout_sec=3.5)
        runner.print_sensor_diagnostic()

        if args.mode == "status":
            if runner.current_pos and runner.base_pos:
                bx, by = gps_enu_delta_m(
                    runner.base_pos[0], runner.base_pos[1],
                    runner.current_pos[0], runner.current_pos[1],
                )
                print("\n💡 Ready-to-run test commands from your current spot:")
                print(f"   • Drive +2.0m East & return : python3 scripts/test_waypoint_navigation.py offset --east 2.0 --north 0.0 --return-start")
                print(f"   • Drive to Base XY (+2m E)  : python3 scripts/test_waypoint_navigation.py base --x {bx+2.0:.2f} --y {by:.2f}")
                tlat, tlon = gps_enu_to_latlon(runner.current_pos[0], runner.current_pos[1], 2.0, 0.0)
                print(f"   • Drive to Global (+2m E)   : python3 scripts/test_waypoint_navigation.py global --lat {tlat:.7f} --lon {tlon:.7f}\n")
            elif runner.current_odom_pos:
                ox, oy = runner.current_odom_pos
                print("\n💡 Ready-to-run LiDAR Odometry test commands from your current spot:")
                print(f"   • Drive +2.0m X & return    : python3 scripts/test_waypoint_navigation.py offset --east 2.0 --north 0.0 --return-start")
                print(f"   • Drive to Odom XY (+2m X)  : python3 scripts/test_waypoint_navigation.py base --x {ox+2.0:.2f} --y {oy:.2f}\n")
            return

        if runner.current_pos is None and runner.current_odom_pos is None:
            print("❌ Cannot start waypoint test: Neither GPS /fix nor LiDAR /odom received.")
            return

        goto_base_type = getattr(ExecuteMotion.Goal, 'TYPE_GOTO_BASE_XY', 4)
        goto_rel_type = getattr(ExecuteMotion.Goal, 'TYPE_GOTO_REL_ENU', 5)

        # LiDAR-only robot execution path (when /fix is absent and /odom is active)
        if runner.current_pos is None and runner.current_odom_pos is not None:
            if args.mode == "global":
                print("❌ Cannot run 'global' (--lat --lon) mode on a LiDAR-only robot without GPS. Use 'base' (--x --y) or 'offset'.")
                return
            elif args.mode == "base":
                runner.execute_waypoint(
                    goto_base_type, args.x, args.y, args.speed, args.rate,
                    target_gps=None, target_base_xy=(args.x, args.y),
                    label=f"LIDAR ODOM_XY ({args.x:+.2f}m, {args.y:+.2f}m)",
                )
                return
            elif args.mode == "offset":
                start_ox, start_oy = runner.current_odom_pos
                target_ox = start_ox + args.east
                target_oy = start_oy + args.north
                ok = runner.execute_waypoint(
                    goto_base_type, target_ox, target_oy, args.speed, args.rate,
                    target_gps=None, target_base_xy=(target_ox, target_oy),
                    label=f"OUTBOUND LIDAR ODOM (+{args.east:.2f}m X, +{args.north:.2f}m Y)",
                )
                if ok and args.return_start:
                    print("🔄 Pausing 2.0s before returning to exact starting LiDAR Odom (X, Y) coordinate...")
                    runner.pause_spin(2.0)
                    runner.execute_waypoint(
                        goto_base_type, start_ox, start_oy, args.speed, args.rate,
                        target_gps=None, target_base_xy=(start_ox, start_oy),
                        label="RETURN TO START LIDAR ODOM_XY",
                    )
                return

        if args.mode == "base":
            if runner.base_pos is None:
                print("❌ Cannot run 'base' mode yet: /gps/base_fix not received. Wait a few seconds for RTCM 1005 or use --frame global.")
                return
            t_gps = gps_enu_to_latlon(runner.base_pos[0], runner.base_pos[1], args.x, args.y)
            runner.execute_waypoint(
                goto_base_type, args.x, args.y, args.speed, args.rate,
                target_gps=t_gps, target_base_xy=(args.x, args.y),
                label=f"BASE_XY ({args.x:+.2f}m E, {args.y:+.2f}m N)",
            )

        elif args.mode == "global":
            de, dn = gps_enu_delta_m(runner.current_pos[0], runner.current_pos[1], args.lat, args.lon)
            runner.execute_waypoint(
                goto_rel_type, de, dn, args.speed, args.rate,
                target_gps=(args.lat, args.lon), target_base_xy=None,
                label=f"GLOBAL ({args.lat:.7f}, {args.lon:.7f})",
            )

        elif args.mode == "offset":
            origin_gps = runner.current_pos
            target_gps = gps_enu_to_latlon(origin_gps[0], origin_gps[1], args.east, args.north)

            if args.frame == "base" and runner.base_pos is not None:
                start_bx, start_by = gps_enu_delta_m(
                    runner.base_pos[0], runner.base_pos[1],
                    origin_gps[0], origin_gps[1],
                )
                target_bx = start_bx + args.east
                target_by = start_by + args.north
                ok = runner.execute_waypoint(
                    goto_base_type, target_bx, target_by, args.speed, args.rate,
                    target_gps=target_gps, target_base_xy=(target_bx, target_by),
                    label=f"OUTBOUND BASE_XY (+{args.east:.2f}m E, +{args.north:.2f}m N)",
                )
                if ok and args.return_start:
                    print("🔄 Pausing 2.0s before returning to exact starting Base (X, Y) coordinate...")
                    runner.pause_spin(2.0)
                    runner.execute_waypoint(
                        goto_base_type, start_bx, start_by, args.speed, args.rate,
                        target_gps=origin_gps, target_base_xy=(start_bx, start_by),
                        label="RETURN TO START BASE_XY",
                    )
            else:
                if args.frame == "base" and runner.base_pos is None:
                    print("⚠️ /gps/base_fix not yet available; falling back to Global ENU mode for this offset test.")
                ok = runner.execute_waypoint(
                    goto_rel_type, args.east, args.north, args.speed, args.rate,
                    target_gps=target_gps, target_base_xy=None,
                    label=f"OUTBOUND GLOBAL (+{args.east:.2f}m E, +{args.north:.2f}m N)",
                )
                if ok and args.return_start and runner.current_pos is not None:
                    print("🔄 Pausing 2.0s before returning to exact starting Global (Lat, Lon) coordinate...")
                    runner.pause_spin(2.0)
                    ret_de, ret_dn = gps_enu_delta_m(
                        runner.current_pos[0], runner.current_pos[1],
                        origin_gps[0], origin_gps[1],
                    )
                    runner.execute_waypoint(
                        goto_rel_type, ret_de, ret_dn, args.speed, args.rate,
                        target_gps=origin_gps, target_base_xy=None,
                        label="RETURN TO START GLOBAL",
                    )
                        target_gps=origin_gps, target_base_xy=None,
                        label="RETURN TO START GLOBAL",
                    )

    except KeyboardInterrupt:
        runner.cancel_active_goal()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
