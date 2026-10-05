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

try:
    from cone_robot_interfaces.action import ExecuteMotion
except ImportError:
    print("[ERROR] cone_robot_interfaces not found. Did you run: source ~/ros2_ws/install/setup.bash ?")
    sys.exit(1)


class SafeActionRunner(Node):
    def __init__(self):
        super().__init__('safe_action_runner')
        self._action_client = ActionClient(self, ExecuteMotion, 'execute_motion')
        self._goal_handle = None
        self._is_done = False

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        self.get_logger().info("Connecting to /execute_motion Action Server...")
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, motion_type: int, distance: float, delta_yaw: float, max_v: float, max_w: float):
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
        sys.stdout.write(
            f"\rExecuting: [{pct:5.1f}%] | Dist Rem: {fb.distance_remaining:4.2f}m | Yaw Rem: {math.degrees(fb.yaw_remaining):5.1f}° | Speed: {fb.current_velocity:4.2f}m/s   "
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
