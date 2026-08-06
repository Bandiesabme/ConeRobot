#!/usr/bin/env python3
"""
ROS 2 Node for Cytron MDD10 Rev 2.0 Dual DC Motor Driver (Tank Steering)
Target Platform: Raspberry Pi 5 / Ubuntu 24.04 Noble (ROS 2 Jazzy)

Subscribes to:
  - /cmd_vel (geometry_msgs/msg/Twist): Target linear and angular velocity.

Hardware Configuration (Cytron MDD10 Rev 2.0 in PWM + DIR mode):
  - Left Motor (Channel A):  PWM1, DIR1
  - Right Motor (Channel B): PWM2, DIR2
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Try importing gpiozero for Raspberry Pi 5 hardware GPIO control
HAS_GPIO = False
try:
    from gpiozero import PWMOutputDevice, OutputDevice
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class MDD10MotorController(Node):

    def __init__(self):
        super().__init__('mdd10_motor_controller')

        # --- Parameters ---
        self.declare_parameter('pwm1_pin', 12)   # Left Motor Speed PWM (GPIO 12 / Pin 32)
        self.declare_parameter('dir1_pin', 24)   # Left Motor Direction (GPIO 24 / Pin 18)
        self.declare_parameter('pwm2_pin', 13)   # Right Motor Speed PWM (GPIO 13 / Pin 33)
        self.declare_parameter('dir2_pin', 25)   # Right Motor Direction (GPIO 25 / Pin 22)
        
        self.declare_parameter('wheel_track', 0.20)     # Distance between wheels in meters
        self.declare_parameter('max_linear_speed', 1.0) # Maximum linear speed (m/s) for scaling
        self.declare_parameter('max_angular_speed', 3.0)# Maximum angular speed (rad/s) for scaling
        
        self.declare_parameter('cmd_timeout', 0.5)     # Fail-safe timeout in seconds
        self.declare_parameter('mock_hardware', False)  # Enable to run node without GPIO hardware

        # Read parameters
        self.pwm1_pin = self.get_parameter('pwm1_pin').value
        self.dir1_pin = self.get_parameter('dir1_pin').value
        self.pwm2_pin = self.get_parameter('pwm2_pin').value
        self.dir2_pin = self.get_parameter('dir2_pin').value

        self.wheel_track = self.get_parameter('wheel_track').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.mock_hardware = self.get_parameter('mock_hardware').value or not HAS_GPIO

        self.last_cmd_time = self.get_clock().now()

        # --- Hardware Initialization ---
        if self.mock_hardware:
            self.get_logger().info("Running in MOCK HARDWARE mode (GPIO calls simulated).")
            self.pwm_left = None
            self.dir_left = None
            self.pwm_right = None
            self.dir_right = None
        else:
            self.get_logger().info(f"Initializing Cytron MDD10 GPIO pins: "
                                   f"Left (PWM:{self.pwm1_pin}, DIR:{self.dir1_pin}), "
                                   f"Right (PWM:{self.pwm2_pin}, DIR:{self.dir2_pin})")
            try:
                self.pwm_left = PWMOutputDevice(self.pwm1_pin, frequency=1000)
                self.dir_left = OutputDevice(self.dir1_pin)
                self.pwm_right = PWMOutputDevice(self.pwm2_pin, frequency=1000)
                self.dir_right = OutputDevice(self.dir2_pin)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize GPIO pins: {e}. Switching to mock hardware.")
                self.mock_hardware = True

        # --- ROS 2 Interfaces ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Watchdog timer to stop motors if communication drops
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("Cytron MDD10 Motor Controller Node started successfully.")

    def cmd_vel_callback(self, msg: Twist):
        """Callback triggered when a /cmd_vel velocity target is received."""
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential Drive Kinematics (Tank Steering)
        # Left speed = v - (w * L / 2)
        # Right speed = v + (w * L / 2)
        v_left = linear_x - (angular_z * self.wheel_track / 2.0)
        v_right = linear_x + (angular_z * self.wheel_track / 2.0)

        # Normalize speeds relative to max linear speed to obtain motor duty cycle [-1.0, 1.0]
        norm_left = max(-1.0, min(1.0, v_left / self.max_linear_speed))
        norm_right = max(-1.0, min(1.0, v_right / self.max_linear_speed))

        self.set_motor_speeds(norm_left, norm_right)

    def set_motor_speeds(self, left_duty: float, right_duty: float):
        """
        Sets speed and direction for left and right channels of Cytron MDD10.
        :param left_duty:  float between -1.0 (full reverse) and 1.0 (full forward)
        :param right_duty: float between -1.0 (full reverse) and 1.0 (full forward)
        """
        # Determine direction and speed for Left Channel
        dir_left_val = left_duty >= 0
        speed_left_val = abs(left_duty)

        # Determine direction and speed for Right Channel
        dir_right_val = right_duty >= 0
        speed_right_val = abs(right_duty)

        if self.mock_hardware:
            self.get_logger().debug(
                f"[MOCK MOTOR] Left: DIR={'FWD' if dir_left_val else 'REV'}, Speed={speed_left_val:.2f} | "
                f"Right: DIR={'FWD' if dir_right_val else 'REV'}, Speed={speed_right_val:.2f}"
            )
            return

        # Drive Physical GPIO Pins
        # Set Direction Pins
        self.dir_left.value = dir_left_val
        self.dir_right.value = dir_right_val

        # Set Speed PWM Duty Cycles (0.0 to 1.0)
        self.pwm_left.value = speed_left_val
        self.pwm_right.value = speed_right_val

    def watchdog_callback(self):
        """Emergency fail-safe: stops motors if no velocity command received recently."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.set_motor_speeds(0.0, 0.0)

    def stop_motors(self):
        """Utility method to safely halt motor movement."""
        self.set_motor_speeds(0.0, 0.0)

    def destroy_node(self):
        """Clean up GPIO resources on shutdown."""
        self.stop_motors()
        if not self.mock_hardware:
            try:
                self.pwm_left.close()
                self.dir_left.close()
                self.pwm_right.close()
                self.dir_right.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MDD10MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motor controller node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
