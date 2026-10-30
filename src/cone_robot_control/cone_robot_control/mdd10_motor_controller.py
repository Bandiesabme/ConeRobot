#!/usr/bin/env python3
"""
ROS 2 Node for Cytron MDD10 Rev 2.0 Dual DC Motor Driver (Tank / Skid Steering)
Target Platform: Raspberry Pi 5 / Ubuntu 24.04 Noble (ROS 2 Jazzy)

Subscribes to:
  - /cmd_vel (geometry_msgs/msg/Twist): Target linear and angular velocity.
  - /battery_state (sensor_msgs/msg/BatteryState): Optional INA219 voltage for PWM compensation.

Hardware Configuration (Cytron MDD10 Rev 2.0 in PWM + DIR mode):
  - Left Motor (Channel A):  PWM1, DIR1
  - Right Motor (Channel B): PWM2, DIR2
"""

import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

# Default to lgpio backend on Raspberry Pi 5 (RP1 chip)
os.environ.setdefault('GPIOZERO_PIN_FACTORY', 'lgpio')

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
        self.declare_parameter('pwm1_pin', 12)             # Left Motor Speed PWM (GPIO 12 / Pin 32)
        self.declare_parameter('dir1_pin', 24)             # Left Motor Direction (GPIO 24 / Pin 18)
        self.declare_parameter('pwm2_pin', 13)             # Right Motor Speed PWM (GPIO 13 / Pin 33)
        self.declare_parameter('dir2_pin', 25)             # Right Motor Direction (GPIO 25 / Pin 22)

        self.declare_parameter('wheel_track', 0.380)       # Effective track width L_eff (meters)
        self.declare_parameter('max_linear_speed', 1.00)   # Physical speed at 100% PWM (m/s)
        self.declare_parameter('max_angular_speed', 3.00)  # Maximum angular speed (rad/s)
        self.declare_parameter('min_pwm', 0.02)            # Static friction deadband duty cycle [0.0 - 0.35]
        self.declare_parameter('left_motor_trim', 1.00)    # Left channel calibration multiplier
        self.declare_parameter('right_motor_trim', 1.00)   # Right channel calibration multiplier
        self.declare_parameter('nominal_battery_v', 12.0)  # Reference voltage for battery compensation
        self.declare_parameter('cmd_timeout', 0.5)         # Fail-safe timeout in seconds

        # Read parameters
        self.pwm1_pin = int(self.get_parameter('pwm1_pin').value)
        self.dir1_pin = int(self.get_parameter('dir1_pin').value)
        self.pwm2_pin = int(self.get_parameter('pwm2_pin').value)
        self.dir2_pin = int(self.get_parameter('dir2_pin').value)

        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.max_linear_speed = max(0.05, float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_pwm = max(0.0, min(0.35, float(self.get_parameter('min_pwm').value)))
        self.left_trim = float(self.get_parameter('left_motor_trim').value)
        self.right_trim = float(self.get_parameter('right_motor_trim').value)
        self.nominal_battery_v = float(self.get_parameter('nominal_battery_v').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.measured_battery_v = self.nominal_battery_v
        self.last_cmd_time = self.get_clock().now()

        # --- Hardware Initialization ---
        self.pwm_left = None
        self.dir_left = None
        self.pwm_right = None
        self.dir_right = None

        if not HAS_GPIO:
            self.get_logger().error("gpiozero is not installed. Motors cannot be initialized.")
        else:
            self.get_logger().info(
                f"Initializing Cytron MDD10 GPIO pins: "
                f"Left (PWM:{self.pwm1_pin}, DIR:{self.dir1_pin}), "
                f"Right (PWM:{self.pwm2_pin}, DIR:{self.dir2_pin}) | "
                f"Track: {self.wheel_track:.3f}m, MaxV: {self.max_linear_speed:.2f}m/s, MinPWM: {self.min_pwm:.2f}"
            )
            try:
                self.pwm_left = PWMOutputDevice(self.pwm1_pin, frequency=1000)
                self.dir_left = OutputDevice(self.dir1_pin)
                self.pwm_right = PWMOutputDevice(self.pwm2_pin, frequency=1000)
                self.dir_right = OutputDevice(self.dir2_pin)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize GPIO pins: {e}")

        # --- ROS 2 Interfaces ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_callback,
            10
        )

        # Watchdog timer to stop motors if communication drops
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("Cytron MDD10 Motor Controller Node started successfully.")

    def _battery_callback(self, msg: BatteryState):
        if msg.voltage and 8.0 <= msg.voltage <= 16.0:
            self.measured_battery_v = float(msg.voltage)

    def _speed_to_duty(self, wheel_speed_mps: float, trim: float, batt_scale: float) -> float:
        """Maps wheel speed (m/s) to PWM duty cycle [-1.0, 1.0] with deadband & trim."""
        if abs(wheel_speed_mps) < 0.003:
            return 0.0
        ratio = min(1.0, abs(wheel_speed_mps) / self.max_linear_speed)
        raw_duty = self.min_pwm + (1.0 - self.min_pwm) * ratio
        scaled_duty = min(1.0, raw_duty * trim * batt_scale)
        return math.copysign(scaled_duty, wheel_speed_mps)

    def cmd_vel_callback(self, msg: Twist):
        """Callback triggered when a /cmd_vel velocity target is received."""
        self.last_cmd_time = self.get_clock().now()

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        # Differential Drive Kinematics (Standard ROS REP-103: +angular_z = CCW / Left)
        # Left speed  = v - (w * L_eff / 2)
        # Right speed = v + (w * L_eff / 2)
        v_left = linear_x - (angular_z * self.wheel_track / 2.0)
        v_right = linear_x + (angular_z * self.wheel_track / 2.0)

        # Preserve curvature ratio if either track exceeds max_linear_speed
        peak_v = max(abs(v_left), abs(v_right))
        if peak_v > self.max_linear_speed:
            scale = self.max_linear_speed / peak_v
            v_left *= scale
            v_right *= scale

        # Battery voltage feedforward normalization (clamped to +/-15% safety window)
        batt_scale = 1.0
        if self.measured_battery_v > 8.0 and self.nominal_battery_v > 8.0:
            batt_scale = max(0.85, min(1.15, self.nominal_battery_v / self.measured_battery_v))

        norm_left = self._speed_to_duty(v_left, self.left_trim, batt_scale)
        norm_right = self._speed_to_duty(v_right, self.right_trim, batt_scale)

        self.set_motor_speeds(norm_left, norm_right)

    def set_motor_speeds(self, left_duty: float, right_duty: float):
        """
        Sets speed and direction for left and right channels of Cytron MDD10.
        :param left_duty:  float between -1.0 (full reverse) and 1.0 (full forward)
        :param right_duty: float between -1.0 (full reverse) and 1.0 (full forward)
        """
        dir_left_val = left_duty >= 0
        speed_left_val = abs(left_duty)

        dir_right_val = right_duty >= 0
        speed_right_val = abs(right_duty)

        if self.dir_left is None or self.pwm_left is None or self.dir_right is None or self.pwm_right is None:
            return

        self.dir_left.value = dir_left_val
        self.dir_right.value = dir_right_val

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
        for device in (self.pwm_left, self.dir_left, self.pwm_right, self.dir_right):
            if device is not None:
                try:
                    device.close()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
