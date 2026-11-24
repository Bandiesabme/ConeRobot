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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

try:
    from .i2c_bus_guard import i2c_bus_lock, read_i2c1_sda_level, recover_i2c1_bus
except (ImportError, ValueError):
    try:
        from cone_robot_control.i2c_bus_guard import i2c_bus_lock, read_i2c1_sda_level, recover_i2c1_bus
    except ImportError:
        import i2c_bus_guard
        i2c_bus_lock = i2c_bus_guard.i2c_bus_lock
        read_i2c1_sda_level = i2c_bus_guard.read_i2c1_sda_level
        recover_i2c1_bus = i2c_bus_guard.recover_i2c1_bus

# Try importing TF2 Broadcaster
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

try:
    import board
    import busio
    from struct import pack_into
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ROTATION_VECTOR,
        Packet,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C

    class ReliableBNO08X_I2C(BNO08X_I2C):
        """
        Subclass of BNO08X_I2C that strictly maintains independent Host TX sequence numbers.
        The stock CircuitPython driver overwrites self._sequence_number with the sensor's
        incoming sequence numbers, causing outgoing SET_FEATURE packets to jump sequence numbers
        (e.g., from 0 directly to 5), which causes the BNO080 sensor hub to reject the packet.
        """
        def __init__(self, i2c_bus, reset=None, address=0x4A, debug=False):
            self._tx_seq = [0, 0, 0, 0, 0, 0]
            self._enabled_features = set()
            super().__init__(i2c_bus, reset=reset, address=address, debug=debug)

        def _dbg(self, *args, **kwargs):
            if not self._debug:
                return
            msg = " ".join(str(a) for a in args)
            if "data_length=0" in msg or "** DONE! **" in msg:
                return
            print("DBG::\t\t", *args, **kwargs)

        def _send_packet(self, channel, data):
            data_length = len(data)
            write_length = data_length + 4

            pack_into("<H", self._data_buffer, 0, write_length)
            self._data_buffer[2] = channel
            self._data_buffer[3] = self._tx_seq[channel]
            for idx, send_byte in enumerate(data):
                self._data_buffer[4 + idx] = send_byte
            packet = Packet(self._data_buffer)
            self._dbg(f"Sending packet (Host TX Seq {self._tx_seq[channel]} on Ch {channel}):")
            self._dbg(packet)
            with self.bus_device_obj as i2c:
                i2c.write(self._data_buffer, end=write_length)

            self._tx_seq[channel] = (self._tx_seq[channel] + 1) % 256

        def enable_feature(self, feature_id, report_interval=50000):
            """Send SET_FEATURE_COMMAND (0xFD) on Channel 2 and register feature."""
            set_feature_report = bytearray(17)
            pack_into(
                "<BBBHIII",
                set_feature_report,
                0,
                0xFD,            # SHTP_REPORT_SET_FEATURE_COMMAND
                feature_id,      # Feature ID
                0,               # Feature flags
                0,               # Change sensitivity
                report_interval, # Report interval in microseconds
                0,               # Batch interval
                0,               # Sensor-specific configuration
            )
            self._send_packet(2, set_feature_report)
            self._enabled_features.add(feature_id)
            time.sleep(0.04)
            for _ in range(5):
                try:
                    self._process_available_packets()
                except Exception:
                    pass
                time.sleep(0.01)

        def _handle_sensor_report(self, report_id, report_bytes):
            self._enabled_features.add(report_id)
            super()._handle_sensor_report(report_id, report_bytes)

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
        self.declare_parameter('reset_pin', -1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('mount_flipped', True)
        self.declare_parameter('use_game_rotation', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # --- Read Parameters ---
        self.i2c_address = self.get_parameter('i2c_address').value
        self.reset_pin = int(self.get_parameter('reset_pin').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mount_flipped = self.get_parameter('mount_flipped').value
        self.use_game_rotation = self.get_parameter('use_game_rotation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # --- Publishers & Broadcasters ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)
        self.compass_pub = self.create_publisher(Float32, '/imu/compass_heading', 10)

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

        # Sensor instances & I2C lockup auto-recovery state
        self.sensor = None
        self.i2c_bus = None
        self.consecutive_errors = 0
        self.consecutive_i2c_errors = 0
        self._last_bus_recovery_time = 0.0
        self._last_reinit_attempt_time = 0.0

        # Defer hardware initialization by 0.1s so ROS 2 node spins immediately
        self._init_timer = self.create_timer(0.1, self._init_hardware_callback)

        # Start IMU publishing timer
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self._publish_imu_data)

        self.get_logger().info(
            f'BNO08x IMU Node initialized (Rate: {self.publish_rate_hz} Hz, '
            f'Flipped: {self.mount_flipped}, Game Rotation: {self.use_game_rotation}, '
            f'Publish TF: {self.publish_tf and TF2_AVAILABLE})'
        )

    def _init_hardware_callback(self):
        """Asynchronously initialize hardware once ROS 2 executor is running."""
        self._init_timer.cancel()
        self._init_sensor()

    def _drain_packets(self, max_packets=10, delay_s=0.01):
        """Drain queued SHTP packets from the BNO08x buffer."""
        if not self.sensor:
            return
        for _ in range(max_packets):
            try:
                self.sensor._process_available_packets()
            except Exception:
                pass
            time.sleep(delay_s)

    def _enable_feature_safe(self, feature_id, feature_name="feature", max_attempts=3):
        """
        Enable a feature on BNO08x with retries.
        """
        if not self.sensor:
            return False

        if hasattr(self.sensor, '_enabled_features') and feature_id in self.sensor._enabled_features:
            return True

        for attempt in range(1, max_attempts + 1):
            try:
                self.sensor.enable_feature(feature_id)
                self.get_logger().info(f'BNO08x: {feature_name} (ID 0x{feature_id:02X}) enabled.')
                return True
            except Exception as e:
                self.get_logger().warn(
                    f'BNO08x: attempt {attempt}/{max_attempts} to enable {feature_name} (ID 0x{feature_id:02X}) failed: {e}'
                )
                self._drain_packets(max_packets=3, delay_s=0.02)
                time.sleep(0.10)

        return False

    def _init_sensor(self):
        """Initialize connection to physical BNO08x sensor with automatic retries and fallback."""
        if not HARDWARE_LIBS_AVAILABLE:
            self.get_logger().error(
                'adafruit-circuitpython-bno08x or hardware bus not available. '
                'Please install dependencies: pip3 install adafruit-circuitpython-bno08x adafruit-blinka'
            )
            self.sensor = None
            return

        max_init_attempts = 2
        for init_attempt in range(1, max_init_attempts + 1):
            try:
                self.get_logger().info(
                    f'Connecting to BNO08x at I2C address 0x{self.i2c_address:02X} '
                    f'(attempt {init_attempt}/{max_init_attempts})...'
                )

                with i2c_bus_lock(1):
                    # If SDA is stuck LOW (GPIO2=0), attempt non-reboot bus recovery before probing
                    if read_i2c1_sda_level() == 0:
                        self.get_logger().warn('I2C1 SDA is stuck LOW (GPIO2=0) — running automatic bus recovery...')
                        ok_rec, rec_detail = recover_i2c1_bus(reset_pin=self.reset_pin)
                        self.get_logger().warn(f'I2C1 recovery result: {rec_detail} (ok={ok_rec})')

                    if self.i2c_bus is None:
                        self.i2c_bus = busio.I2C(board.SCL, board.SDA, frequency=100000)

                    self.sensor = ReliableBNO08X_I2C(self.i2c_bus, address=self.i2c_address)
                    # Allow BNO08x SH-2 internal processor to settle after soft-reset and drain boot packets
                    time.sleep(0.45)
                    self._drain_packets(max_packets=15, delay_s=0.01)

                    # Determine primary and fallback rotation features
                    if self.use_game_rotation:
                        primary_feat = BNO_REPORT_GAME_ROTATION_VECTOR
                        primary_name = "Game Rotation Vector (6-DOF)"
                        fallback_feat = BNO_REPORT_ROTATION_VECTOR
                        fallback_name = "Absolute Rotation Vector (9-DOF / Compass)"
                    else:
                        primary_feat = BNO_REPORT_ROTATION_VECTOR
                        primary_name = "Absolute Rotation Vector (9-DOF / Compass)"
                        fallback_feat = BNO_REPORT_GAME_ROTATION_VECTOR
                        fallback_name = "Game Rotation Vector (6-DOF)"

                    rot_ok = self._enable_feature_safe(primary_feat, primary_name)
                    if not rot_ok:
                        self.get_logger().warn(
                            f'{primary_name} failed. Attempting fallback to {fallback_name}...'
                        )
                        rot_ok = self._enable_feature_safe(fallback_feat, fallback_name)
                        if rot_ok:
                            self.use_game_rotation = not self.use_game_rotation
                            self.get_logger().info(
                                f'Successfully activated fallback {fallback_name} (use_game_rotation={self.use_game_rotation}).'
                            )
                        else:
                            raise RuntimeError(f'Could not enable {primary_name} or {fallback_name}.')

                    time.sleep(0.05)
                    self._enable_feature_safe(BNO_REPORT_GYROSCOPE, "Gyroscope", max_attempts=3)
                    time.sleep(0.05)
                    self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, "Accelerometer", max_attempts=3)

                self.consecutive_errors = 0
                self.consecutive_i2c_errors = 0
                self.get_logger().info('✅ BNO08x hardware initialized successfully.')
                return

            except Exception as e:
                self.get_logger().error(f'BNO08x init attempt {init_attempt}/{max_init_attempts} failed: {e}')
                if init_attempt < max_init_attempts:
                    with i2c_bus_lock(1):
                        if read_i2c1_sda_level() == 0 or self.reset_pin >= 0:
                            recover_i2c1_bus(reset_pin=self.reset_pin)
                    time.sleep(1.0)

        self.get_logger().error('❌ BNO08x hardware failed to initialize. Will retry automatically every 5s.')
        self.sensor = None
        self._last_reinit_attempt_time = time.monotonic()

    def _publish_imu_data(self):
        """Timer callback reading sensor, publishing ROS 2 IMU messages and TF."""
        if self.sensor is None:
            now_m = time.monotonic()
            if (now_m - self._last_reinit_attempt_time) >= 5.0:
                self._last_reinit_attempt_time = now_m
                self._init_sensor()
            return

        now = self.get_clock().now()
        stamp = now.to_msg()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance = self.orientation_covariance
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        try:
            # Atomic cross-process lock keeps INA219 (system_info_node) from interleaving SHTP packets
            with i2c_bus_lock(1):
                if self.use_game_rotation:
                    quat_raw = self.sensor.game_quaternion
                else:
                    quat_raw = self.sensor.quaternion

                gyro_raw = self.sensor.gyro
                accel_raw = self.sensor.acceleration

            if quat_raw is None:
                return

            qx, qy, qz, qw = quat_raw[0], quat_raw[1], quat_raw[2], quat_raw[3]
            if (qx * qx + qy * qy + qz * qz + qw * qw) < 0.0001:
                return
            gx, gy, gz = (gyro_raw[0], gyro_raw[1], gyro_raw[2]) if gyro_raw is not None else (0.0, 0.0, 0.0)
            ax, ay, az = (accel_raw[0], accel_raw[1], accel_raw[2]) if accel_raw is not None else (0.0, 0.0, 9.81)

            # Apply Flip Correction if mounted upside down
            if self.mount_flipped:
                # Right-multiply by 180-deg X-roll (body-frame sensor-to-robot transform: q_w_r = q_w_s * q_s_r).
                # Left-multiplying flips the world Z axis and inverts the yaw sign (+CCW becomes -CW).
                q_corrected = quaternion_multiply([qx, qy, qz, qw], self.q_flip)
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
            self.consecutive_i2c_errors = 0

        except (RuntimeError, OSError, ValueError, KeyError) as e:
            self.consecutive_errors += 1
            err_str = str(e)
            if isinstance(e, OSError):
                self.consecutive_i2c_errors += 1
                now_m = time.monotonic()
                if self.consecutive_i2c_errors >= 5 and (now_m - self._last_bus_recovery_time) >= 8.0:
                    self._last_bus_recovery_time = now_m
                    self.consecutive_i2c_errors = 0
                    self.get_logger().warn(
                        f'Repeated BNO08x I2C OSError ({e}) — attempting automatic I2C1 bus recovery & re-init...'
                    )
                    with i2c_bus_lock(1):
                        ok_rec, rec_detail = recover_i2c1_bus(reset_pin=self.reset_pin)
                    self.get_logger().warn(f'I2C1 auto-recovery: {rec_detail} (ok={ok_rec})')
                    self.sensor = None
                    self._init_sensor()
                    return

            # Self-healing: if the BNO08x finished soft-reset after the initial enable_feature
            # packet or reset mid-run, automatically re-send SET_FEATURE_COMMAND (0xFD)
            if "report found, is it enabled" in err_str and self.sensor is not None:
                if self.consecutive_errors in (5, 25, 50) or (self.consecutive_errors % 50 == 25):
                    try:
                        with i2c_bus_lock(1):
                            self._drain_packets(max_packets=8, delay_s=0.005)
                            feat = BNO_REPORT_GAME_ROTATION_VECTOR if self.use_game_rotation else BNO_REPORT_ROTATION_VECTOR
                            self.sensor.enable_feature(feat)
                            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
                            self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                    except Exception:
                        pass
            if self.consecutive_errors % 50 == 1 and self.consecutive_errors > 1:
                self.get_logger().warn(
                    f'Transient BNO08x I2C read error (suppressed, count={self.consecutive_errors}): {e}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = BNO08xNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
