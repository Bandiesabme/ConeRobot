#!/usr/bin/env python3
"""
Simple Cross-Platform Teleop Keyboard Node for ROS 2
Can be run on Laptop (Windows/Linux/WSL) or Raspberry Pi to send /cmd_vel commands.
"""

import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Windows vs POSIX terminal input handling
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

MSG = """
==================================================
  Cone Robot Teleop Keyboard Controller
==================================================
  Control Keys:
        W
    A   S   D
  
  W / S : Increase / Decrease Forward Speed
  A / D : Turn Left / Right
  SPACE / K : Emergency STOP (Zero Speed)
  Q / Z : Increase / Decrease Speed Steps

  CTRL+C : Quit
==================================================
"""

class TeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.2    # Default m/s
        self.angular_speed = 0.5   # Default rad/s

        self.linear_step = 0.05
        self.angular_step = 0.1

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.get_logger().info("Teleop Keyboard Node initialized.")
        print(MSG)
        self.print_status()

    def print_status(self):
        print(f"\rCurrent Target -> Linear: {self.target_linear:.2f} m/s | Angular: {self.target_angular:.2f} rad/s", end="", flush=True)

    def publish_cmd(self, linear, angular):
        self.target_linear = round(linear, 2)
        self.target_angular = round(angular, 2)

        twist = Twist()
        twist.linear.x = float(self.target_linear)
        twist.angular.z = float(self.target_angular)
        self.publisher.publish(twist)
        self.print_status()

    def process_key(self, key):
        key = key.lower()
        if key == 'w':
            self.publish_cmd(self.target_linear + self.linear_step, self.target_angular)
        elif key == 's':
            self.publish_cmd(self.target_linear - self.linear_step, self.target_angular)
        elif key == 'a':
            self.publish_cmd(self.target_linear, self.target_angular + self.angular_step)
        elif key == 'd':
            self.publish_cmd(self.target_linear, self.target_angular - self.angular_step)
        elif key == ' ' or key == 'k':
            self.publish_cmd(0.0, 0.0)
        elif key == 'q':
            self.linear_step += 0.01
            self.angular_step += 0.05
            print(f"\n[INFO] Speed step increased -> Lin: {self.linear_step:.2f}, Ang: {self.angular_step:.2f}")
        elif key == 'z':
            self.linear_step = max(0.01, self.linear_step - 0.01)
            self.angular_step = max(0.05, self.angular_step - 0.05)
            print(f"\n[INFO] Speed step decreased -> Lin: {self.linear_step:.2f}, Ang: {self.angular_step:.2f}")


def get_key_win():
    """Read single character on Windows."""
    return msvcrt.getch().decode('utf-8', errors='ignore')


def get_key_posix(settings):
    """Read single character on Linux / POSIX."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    settings = None
    if sys.platform != 'win32':
        settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            if sys.platform == 'win32':
                if msvcrt.kbhit():
                    key = get_key_win()
                    if key == '\x03':  # Ctrl+C
                        break
                    node.process_key(key)
            else:
                key = get_key_posix(settings)
                if key == '\x03':  # Ctrl+C
                    break
                if key:
                    node.process_key(key)

            rclpy.spin_once(node, timeout_sec=0.05)

    except Exception as e:
        print(f"\n[ERROR] Teleop error: {e}")
    finally:
        node.publish_cmd(0.0, 0.0)
        if sys.platform != 'win32' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nExited Teleop Keyboard Controller.")


if __name__ == '__main__':
    main()
