#!/usr/bin/env python3
"""
BNO08x (BNO080 / BNO085) IMU ROS 2 Driver Node
Supports MikroE Click / Adafruit / SparkFun breakout boards over I2C.
Includes upside-down / flipped mounting correction, optional dynamic TF broadcasting,
and fault-tolerant reading.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Float32

# Try importing SMBus for ultra-fast direct I2C register reads without heavy wrappers
SMBUS_AVAILABLE = False
try:
    import smbus2 as smbus
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False

# Try importing TF2 Broadcaster
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

# Try importing hardware libraries
try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    HARDWARE_LIBS_AVAILABLE = True
except (ImportError, NotImplementedError):
    HARDWARE_LIBS_AVAILABLE = False


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions [x, y, z, w].
    q_out = q1 * q2
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quaternion_to_yaw_deg(x, y, z, w):
    """Calculate yaw (heading) in degrees from quaternion [x, y, z, w]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)
    return (yaw_deg + 360.0) % 360.0


class BNO08xNode(Node):
    """ROS 2 Node interfacing BNO08x IMU via I2C with flip-correction and TF."""

    def __init__(self):
        super().__init__('bno08x_node')

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter('i2c_address', 74)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('mount_flipped', True)
        self.declare_parameter('use_game_rotation', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('mock_hardware', False)

        # --- Read Parameters ---
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mount_flipped = self.get_parameter('mount_flipped').value
        self.use_game_rotation = self.get_parameter('use_game_rotation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.mock_hardware = self.get_parameter('mock_hardware').value

        # --- INA219 Battery Monitor Parameters (Shares I2C Bus, 0 MB Extra RAM) ---
        self.declare_parameter('enable_battery_monitor', True)
        self.declare_parameter('battery_i2c_address', 64)  # 0x40 default
        self.declare_parameter('battery_publish_rate_hz', 1.0)
        self.declare_parameter('battery_min_v', 10.5)  # 3S cutoff (3.5V/cell)
        self.declare_parameter('battery_max_v', 12.6)  # 3S full (4.2V/cell)
        self.declare_parameter('battery_low_warn_v', 11.1) # 3S warning (3.7V/cell)

        self.enable_battery_monitor = bool(self.get_parameter('enable_battery_monitor').value)
        self.battery_i2c_address = int(self.get_parameter('battery_i2c_address').value)
        self.battery_publish_rate_hz = float(self.get_parameter('battery_publish_rate_hz').value)
        self.battery_min_v = float(self.get_parameter('battery_min_v').value)
        self.battery_max_v = float(self.get_parameter('battery_max_v').value)
        self.battery_low_warn_v = float(self.get_parameter('battery_low_warn_v').value)

        # --- Publishers & Broadcasters ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)

        if self.enable_battery_monitor:
            self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        else:
            self.battery_pub = None

        if self.publish_tf and TF2_AVAILABLE:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            if self.publish_tf and not TF2_AVAILABLE:
                self.get_logger().warn('tf2_ros is not installed. Skipping dynamic TF broadcast.')

        # 180-deg roll rotation quaternion [x, y, z, w] for flipped mounting
        self.q_flip = [1.0, 0.0, 0.0, 0.0]

        # Standard Covariances
        self.orientation_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-4
        ]
        self.angular_velocity_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-4
        ]
        self.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        ]

        # Sensor instances
        self.sensor = None
        self.i2c_bus = None
        self.smbus_bus = None
        self.consecutive_errors = 0
        self._last_battery_warn_time = 0.0

        # Initialize Hardware
        self._init_sensor()
        if self.enable_battery_monitor:
            self._init_battery_monitor()

        # Start IMU publishing timer
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self._publish_imu_data)

        # Start Battery publishing timer (Low frequency, e.g. 1 Hz)
        if self.enable_battery_monitor:
            batt_period = 1.0 / max(0.1, self.battery_publish_rate_hz)
            self.battery_timer = self.create_timer(batt_period, self._publish_battery_data)
            self.get_logger().info(
                f'INA219 Battery Monitor scheduled (Rate: {self.battery_publish_rate_hz} Hz, '
                f'I2C: 0x{self.battery_i2c_address:02X}, Range: {self.battery_min_v}V - {self.battery_max_v}V)'
            )
        self.get_logger().info(
            f'BNO08x IMU Node initialized (Rate: {self.publish_rate_hz} Hz, '
            f'Flipped: {self.mount_flipped}, Game Rotation: {self.use_game_rotation}, '
            f'Publish TF: {self.publish_tf and TF2_AVAILABLE})'
        )

    def _init_sensor(self):
        """Initialize connection to physical BNO08x sensor."""
        if self.mock_hardware or not HARDWARE_LIBS_AVAILABLE:
            if not self.mock_hardware:
                self.get_logger().warn(
                    'adafruit-circuitpython-bno08x or hardware bus not available. Running in MOCK mode.'
                )
            self.mock_hardware = True
            return

        try:
            self.get_logger().info(
                f'Connecting to BNO08x at I2C address 0x{self.i2c_address:02X}...'
            )
            i2c = busio.I2C(board.SCL, board.SDA)
            self.i2c_bus = i2c
            self.sensor = BNO08X_I2C(i2c, address=self.i2c_address)
            time.sleep(0.5)

            # Enable desired SHTP reports
            if self.use_game_rotation:
                self.sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            else:
                self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.1)

            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.1)

            self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
            time.sleep(0.1)

            self.get_logger().info('BNO08x hardware initialized successfully.')
            self.mock_hardware = False
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO08x over I2C: {e}')
            self.get_logger().warn('Falling back to MOCK hardware mode for stability.')
            self.mock_hardware = True

    def _init_battery_monitor(self):
        """Initialize INA219 battery monitor interface on the shared I2C bus."""
        if self.mock_hardware:
            self.get_logger().info('INA219 Battery Monitor: running in MOCK mode.')
            return

        # Method 1: SMBus (native Linux kernel I2C locking - fastest & most reliable)
        if SMBUS_AVAILABLE:
            try:
                self.smbus_bus = smbus.SMBus(1)
                self.get_logger().info(
                    f'INA219 Battery Monitor initialized via smbus on bus 1 (address: 0x{self.battery_i2c_address:02X})'
                )
                return
            except Exception as e:
                self.get_logger().warn(f'Could not open smbus(1): {e}. Will attempt busio fallback.')

        # Method 2: busio.I2C fallback (shared with BNO08x)
        if self.i2c_bus is not None:
            self.get_logger().info(
                f'INA219 Battery Monitor ready via CircuitPython busio.I2C fallback (address: 0x{self.battery_i2c_address:02X})'
            )
        else:
            self.get_logger().warn('No I2C interface available for INA219 battery monitor.')

    def _publish_battery_data(self):
        """Low-frequency (1 Hz) timer reading INA219 bus voltage and publishing BatteryState."""
        if not self.enable_battery_monitor or self.battery_pub is None:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Mock Mode
        if self.mock_hardware:
            mock_voltage = 12.25  # Simulated healthy 3S LiPo
            span = max(self.battery_max_v - self.battery_min_v, 0.1)
            pct = max(0.0, min(1.0, (mock_voltage - self.battery_min_v) / span))
            msg.voltage = float(mock_voltage)
            msg.percentage = float(pct)
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            self.battery_pub.publish(msg)
            return

        voltage_v = None

        # 1. Read via SMBus if available
        if self.smbus_bus is not None:
            try:
                # Register 0x02 is Bus Voltage
                raw = self.smbus_bus.read_word_data(self.battery_i2c_address, 0x02)
                # INA219 transmits big-endian; smbus read_word_data assumes little-endian
                raw_swapped = ((raw & 0xFF) << 8) | (raw >> 8)
                raw_bits = (raw_swapped >> 3) & 0x1FFF
                voltage_v = raw_bits * 0.004  # LSB = 4 mV
            except Exception as e:
                self.get_logger().warn(
                    f'INA219 I2C read error (smbus): {e}',
                    throttle_duration_sec=10.0
                )

        # 2. Fallback to busio.I2C
        if voltage_v is None and self.i2c_bus is not None:
            try:
                if self.i2c_bus.try_lock():
                    try:
                        buf = bytearray(2)
                        self.i2c_bus.writeto_then_readfrom(self.battery_i2c_address, bytes([0x02]), buf)
                        raw = (buf[0] << 8) | buf[1]
                        raw_bits = (raw >> 3) & 0x1FFF
                        voltage_v = raw_bits * 0.004
                    finally:
                        self.i2c_bus.unlock()
            except Exception as e:
                self.get_logger().warn(
                    f'INA219 I2C read error (busio): {e}',
                    throttle_duration_sec=10.0
                )

        if voltage_v is not None:
            # Calculate charge percentage (0.0 - 1.0)
            span = max(self.battery_max_v - self.battery_min_v, 0.1)
            pct = max(0.0, min(1.0, (voltage_v - self.battery_min_v) / span))

            msg.voltage = float(round(voltage_v, 2))
            msg.percentage = float(round(pct, 2))

            if voltage_v <= self.battery_min_v:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            elif voltage_v <= self.battery_low_warn_v:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            else:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

            self.battery_pub.publish(msg)

            # Warning log on low battery (throttled every 15s)
            curr_time = time.time()
            if voltage_v <= self.battery_low_warn_v and (curr_time - self._last_battery_warn_time > 15.0):
                self.get_logger().warn(
                    f'⚠️ LOW BATTERY WARNING: {voltage_v:.2f}V ({int(pct*100)}%). Recharge recommended!'
                )
                self._last_battery_warn_time = curr_time

    def _publish_imu_data(self):
        """Timer callback reading sensor, publishing ROS 2 IMU messages and TF."""
        now = self.get_clock().now()
        stamp = now.to_msg()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance = self.orientation_covariance
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        if self.mock_hardware:
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81

            self.imu_pub.publish(msg)
            heading_msg = Float32()
            heading_msg.data = 0.0
            self.heading_pub.publish(heading_msg)
            return

        try:
            if self.use_game_rotation:
                quat_raw = self.sensor.game_quaternion
            else:
                quat_raw = self.sensor.quaternion

            gyro_raw = self.sensor.gyro
            accel_raw = self.sensor.acceleration

            if quat_raw is None or gyro_raw is None or accel_raw is None:
                return

            qx, qy, qz, qw = quat_raw[0], quat_raw[1], quat_raw[2], quat_raw[3]
            gx, gy, gz = gyro_raw[0], gyro_raw[1], gyro_raw[2]
            ax, ay, az = accel_raw[0], accel_raw[1], accel_raw[2]

            # Apply Flip Correction if mounted upside down
            if self.mount_flipped:
                q_corrected = quaternion_multiply(self.q_flip, [qx, qy, qz, qw])
                msg.orientation.x = float(q_corrected[0])
                msg.orientation.y = float(q_corrected[1])
                msg.orientation.z = float(q_corrected[2])
                msg.orientation.w = float(q_corrected[3])

                msg.angular_velocity.x = float(gx)
                msg.angular_velocity.y = float(-gy)
                msg.angular_velocity.z = float(-gz)

                msg.linear_acceleration.x = float(ax)
                msg.linear_acceleration.y = float(-ay)
                msg.linear_acceleration.z = float(-az)
            else:
                msg.orientation.x = float(qx)
                msg.orientation.y = float(qy)
                msg.orientation.z = float(qz)
                msg.orientation.w = float(qw)

                msg.angular_velocity.x = float(gx)
                msg.angular_velocity.y = float(gy)
                msg.angular_velocity.z = float(gz)

                msg.linear_acceleration.x = float(ax)
                msg.linear_acceleration.y = float(ay)
                msg.linear_acceleration.z = float(az)

            # Publish IMU Message
            self.imu_pub.publish(msg)

            # Calculate and Publish 2D Yaw Heading in Degrees (0 - 360 deg)
            yaw_deg = quaternion_to_yaw_deg(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            heading_msg = Float32()
            heading_msg.data = float(yaw_deg)
            self.heading_pub.publish(heading_msg)

            # Broadcast Dynamic TF (odom -> base_link) for 3D Visualizer if available
            if self.tf_broadcaster:
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self.parent_frame_id
                t.child_frame_id = self.child_frame_id
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = msg.orientation.x
                t.transform.rotation.y = msg.orientation.y
                t.transform.rotation.z = msg.orientation.z
                t.transform.rotation.w = msg.orientation.w
                self.tf_broadcaster.sendTransform(t)

            self.consecutive_errors = 0

        except (RuntimeError, OSError, ValueError, KeyError) as e:
            self.consecutive_errors += 1
            if self.consecutive_errors % 50 == 1:
                self.get_logger().warn(
                    f'Transient BNO08x I2C read error (suppressed, count={self.consecutive_errors}): {e}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = BNO08xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()