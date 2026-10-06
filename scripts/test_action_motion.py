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


class SafeActionRunner(Node):
    def __init__(self):
        super().__init__('safe_action_runner')
        self._action_client = ActionClient(self, ExecuteMotion, 'execute_motion')
        self._goal_handle = None
        self._is_done = False

        # Real-time GPS RTK Fix & Position Monitor
        self.gps_status_sub = self.create_subscription(String, '/gps/status', self._gps_status_cb, 10)
        self.gps_fix_sub = self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)
        self.current_fix_quality = "UNKNOWN"
        self.num_sats = 0
        self.hdop = 99.99
        self.initial_fix_quality = None
        self.drop_events = []
        self.motion_start_time = None

        self.current_pos = None
        self.start_pos = None
        self.target_distance = 0.0
        self.active_motion_type = 0

    def _fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.current_pos = (msg.latitude, msg.longitude)
            if self.motion_start_time and self.start_pos is None:
                self.start_pos = (msg.latitude, msg.longitude)

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

        # Real-time GPS status badge
        if "RTK FIX" in self.current_fix_quality:
            gps_badge = "🟢 RTK FIX"
        elif "RTK FLOAT" in self.current_fix_quality:
            gps_badge = "🟡 RTK FLOAT"
        elif "3D" in self.current_fix_quality:
            gps_badge = "🔵 3D FIX"
        else:
            gps_badge = f"⚪ {self.current_fix_quality}"

        sat_str = f"|{self.num_sats}s" if self.num_sats > 0 else ""
        sys.stdout.write(
            f"\rExecuting: [{pct:5.1f}%] | Dist Rem: {fb.distance_remaining:4.2f}m | Yaw Rem: {math.degrees(fb.yaw_remaining):5.1f}° | Speed: {fb.current_velocity:4.2f}m/s | GPS: [{gps_badge}{sat_str}]   "
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

            # Print Linear Displacement Accuracy Benchmark for Straight motions
            if self.start_pos and self.current_pos and self.active_motion_type == 0 and self.target_distance > 0.05:
                actual_gps_dist = gps_distance_m(self.start_pos[0], self.start_pos[1], self.current_pos[0], self.current_pos[1])
                err_m = actual_gps_dist - self.target_distance
                err_cm = abs(err_m) * 100.0

                print("\n" + "=" * 65)
                print(" 🎯 LINEAR DISPLACEMENT ACCURACY BENCHMARK (RTK GPS)")
                print("=" * 65)
                print(f" Commanded Target Distance : {self.target_distance:.3f} m")
                print(f" Measured RTK GPS Distance : {actual_gps_dist:.3f} m")
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

            # Print GPS Performance & Stability Report
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
        if args.command == "circle":
            r = abs(args.radius)
            circumference = 2.0 * math.pi * r
            sign = 1.0 if args.direction == "left" else -1.0
            delta_yaw = sign * 2.0 * math.pi
            w_max = args.speed / r
            if runner.send_goal(2, circumference, delta_yaw, args.speed, w_max):
                runner.wait_for_result()

        elif args.command == "straight":
            if runner.send_goal(0, args.distance, 0.0, args.speed, 0.0):
                runner.wait_for_result()

        elif args.command == "rotate":
            yaw_rad = math.radians(args.deg)
            if runner.send_goal(1, 0.0, yaw_rad, 0.0, args.rate):
                runner.wait_for_result()

        elif args.command == "stop":
            print("Sending TYPE_STOP...")
            runner.send_goal(3, 0.0, 0.0, 0.0, 0.0)
            runner.wait_for_result()

    except KeyboardInterrupt:
        runner.cancel_active_goal()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
