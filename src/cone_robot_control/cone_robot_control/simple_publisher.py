#!/usr/bin/env python3
"""
Simple Laptop Publisher Node (ROS 2)
Publishes target speed and steering direction to the Raspberry Pi 5 motor controller.

Topic: /cmd_vel (geometry_msgs/msg/Twist)
  - speed (linear.x):  Forward/Backward speed in m/s (e.g., 0.5 = forward, -0.5 = reverse, 0 = stop)
  - direction (angular.z): Steering rate in rad/s (e.g., 0.5 = turn left, -0.5 = turn right, 0 = straight)
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleLaptopPublisher(Node):

    def __init__(self):
        super().__init__('simple_laptop_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Simple Laptop Speed & Direction Publisher Node Started.")

    def publish_move_command(self, speed: float, direction: float):
        """
        Publishes a movement command over ROS 2.
        
        :param speed: Forward/backward speed in m/s (-1.0 to 1.0)
        :param direction: Steering turn rate in rad/s (+ = Left, - = Right)
        """
        msg = Twist()
        msg.linear.x = float(speed)      # Speed
        msg.angular.z = float(direction)  # Steering Direction
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent Command -> Speed: {speed:.2f} m/s | Direction/Turn: {direction:.2f} rad/s")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaptopPublisher()

    try:
        print("\n=== ROS 2 Laptop Command Sequence Demo ===")
        print("Sending move commands to Raspberry Pi 5...\n")

        # Example 1: Drive Forward at 0.4 m/s straight for 2 seconds
        node.get_logger().info("Action 1: Moving Forward Straight...")
        for _ in range(20): # 20 * 0.1s = 2.0s
            node.publish_move_command(speed=0.4, direction=0.0)
            time.sleep(0.1)

        # Example 2: Turn Left while moving forward at 0.3 m/s for 1.5 seconds
        node.get_logger().info("Action 2: Turning Left...")
        for _ in range(15):
            node.publish_move_command(speed=0.3, direction=0.8)
            time.sleep(0.1)

        # Example 3: Turn Right while moving forward at 0.3 m/s for 1.5 seconds
        node.get_logger().info("Action 3: Turning Right...")
        for _ in range(15):
            node.publish_move_command(speed=0.3, direction=-0.8)
            time.sleep(0.1)

        # Example 4: Stop the robot
        node.get_logger().info("Action 4: Stopping Robot...")
        node.publish_move_command(speed=0.0, direction=0.0)

        print("\nSequence completed!")

    except KeyboardInterrupt:
        node.get_logger().info("Stopping publisher...")
        node.publish_move_command(speed=0.0, direction=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
