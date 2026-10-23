#!/usr/bin/env python3
"""
BNO08x & INA219 Hardware Diagnostic & Calibration Tool
-------------------------------------------------------
Directly tests the I2C connection to the MikroE BNO08x IMU (0x4A)
and INA219 Battery Monitor (0x40) without needing to launch the entire ROS 2 stack.

Usage:
    python3 tools/test_bno08x_hardware.py
"""

import sys
import time
import math

print("=" * 65)
print("  🧭 CONE ROBOT — BNO08x IMU & INA219 HARDWARE DIAGNOSTICS")
print("=" * 65)

# 1. Check Python Libraries
print("\n[Step 1/4] Checking required Python packages...")
missing_pkgs = []
try:
    import board
    import busio
    print("  ✓ CircuitPython Blinka & busio found.")
except ImportError:
    missing_pkgs.append("adafruit-blinka")

try:
    from struct import pack_into
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ROTATION_VECTOR,
        Packet,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    print("  ✓ adafruit-circuitpython-bno08x found.")
except ImportError:
    missing_pkgs.append("adafruit-circuitpython-bno08x")


class ReliableBNO08X_I2C(BNO08X_I2C):
    """
    Subclass of BNO08X_I2C that strictly maintains independent Host TX sequence numbers.
    The stock CircuitPython driver overwrites self._sequence_number with the sensor's
    incoming sequence numbers, causing outgoing SET_FEATURE packets to jump sequence numbers
    (e.g., from 0 directly to 5), which causes the BNO080 sensor hub to reject the packet.
    """
    def __init__(self, i2c_bus, reset=None, address=0x4A, debug=False):
        self._tx_seq = [0, 0, 0, 0, 0, 0]
        super().__init__(i2c_bus, reset=reset, address=address, debug=debug)

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

    def _handle_sensor_report(self, report_id, report_bytes):
        # Acknowledge feature active if data packets begin streaming on Channel 3
        self._enabled_features.add(report_id)
        super()._handle_sensor_report(report_id, report_bytes)

try:
    import smbus2 as smbus
    print("  ✓ smbus2 found.")
except ImportError:
    try:
        import smbus
        print("  ✓ smbus found.")
    except ImportError:
        missing_pkgs.append("smbus")

if missing_pkgs:
    print(f"\n❌ Missing required packages: {', '.join(missing_pkgs)}")
    print("   Run: pip3 install --break-system-packages " + " ".join(missing_pkgs))
    sys.exit(1)


# 2. Probe I2C Bus Devices
print("\n[Step 2/4] Scanning I2C Bus 1 (Pins 3 & 5: GPIO 2 SDA, GPIO 3 SCL)...")
detected_addrs = []
try:
    bus = smbus.SMBus(1)
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            detected_addrs.append(addr)
        except OSError:
            pass
    bus.close()
    hex_addrs = [f"0x{a:02X}" for a in detected_addrs]
    print(f"  Detected I2C devices: {', '.join(hex_addrs) if hex_addrs else 'None'}")
    
    if 0x4A in detected_addrs:
        print("  ✓ BNO08x detected at address 0x4A!")
    elif 0x4B in detected_addrs:
        print("  ⚠️ BNO08x detected at address 0x4B (alternate address).")
    else:
        print("  ❌ BNO08x NOT detected at 0x4A. Check 3.3V, GND, SDA, and SCL wiring.")

    if 0x40 in detected_addrs:
        print("  ✓ INA219 Battery Monitor detected at address 0x40!")
    else:
        print("  ⚠️ INA219 not detected at address 0x40.")
except Exception as e:
    print(f"  Could not run low-level SMBus scan: {e}")


debug_mode = "--debug" in sys.argv
if debug_mode:
    print("  🔍 Debug logging enabled (SHTP packet trace active).")

# 3. Connect to BNO08x & Test SHTP Features
print("\n[Step 3/4] Initializing BNO08x over I2C...")
i2c = None
sensor = None
try:
    target_addr = 0x4B if (0x4B in detected_addrs and 0x4A not in detected_addrs) else 0x4A
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    print(f"  Connecting to BNO08x at I2C address 0x{target_addr:02X}...")
    sensor = ReliableBNO08X_I2C(i2c, address=target_addr, debug=debug_mode)
    print(f"  ✓ SHTP transport established with BNO08x at 0x{target_addr:02X}.")
    
    # Allow sensor hub to settle after ID query
    time.sleep(0.3)

    # Test Enabling Rotation Reports
    print("  Enabling Game Rotation Vector (6-DOF, Feature 0x08)...")
    game_ok = False
    for attempt in range(1, 6):
        try:
            sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            game_ok = True
            print(f"  ✓ Game Rotation Vector enabled on attempt {attempt}!")
            break
        except Exception as e:
            if debug_mode or attempt == 5:
                print(f"    Attempt {attempt} failed: {e}")
            for _ in range(5):
                try:
                    sensor._process_available_packets()
                except Exception:
                    pass
                time.sleep(0.02)
            time.sleep(0.15)
    
    if not game_ok:
        print("  ⚠️ Could not enable Game Rotation. Trying Absolute Rotation Vector (9-DOF / Compass, Feature 0x05)...")
        for attempt in range(1, 6):
            try:
                sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                print(f"  ✓ Absolute Rotation Vector enabled on attempt {attempt}!")
                break
            except Exception as e:
                if debug_mode or attempt == 5:
                    print(f"    Attempt {attempt} failed: {e}")
                time.sleep(0.15)

    # Enable Gyro & Accel
    try:
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("  ✓ Gyroscope report enabled.")
    except Exception as e:
        print(f"  ⚠️ Gyroscope report warning: {e}")

    try:
        sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        print("  ✓ Accelerometer report enabled.")
    except Exception as e:
        print(f"  ⚠️ Accelerometer report warning: {e}")

except Exception as e:
    print(f"\n❌ Failed to connect to BNO08x: {e}")
    print("\nTroubleshooting tips:")
    print("1. Check wiring: SDA -> Pin 3 (GPIO 2), SCL -> Pin 5 (GPIO 3), 3.3V -> Pin 1, GND -> Pin 6.")
    print("2. If the chip was recently running, power-cycle the robot (unplug LiPo / power cut) for a cold reset.")
    print("3. Check /boot/firmware/config.txt to ensure 'dtparam=i2c_arm=on,i2c_arm_baudrate=100000' is set.")
    sys.exit(1)


# 4. Live Data Stream
print("\n[Step 4/4] Streaming live sensor data for 10 seconds...")
print("Rotate or tilt the robot to verify heading changes!\n")
print(f"{'Time':^8} | {'Heading':^9} | {'Pitch':^8} | {'Roll':^8} | {'Gyro Z':^10} | {'Battery':^9}")
print("-" * 65)

def quat_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    # Yaw (z-axis rotation / heading)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = (math.degrees(math.atan2(siny_cosp, cosy_cosp)) + 360.0) % 360.0

    return yaw, pitch, roll

start_time = time.time()
sample_count = 0
last_print = 0.0

while time.time() - start_time < 10.0:
    time.sleep(0.04)  # ~25 Hz sampling
    sample_count += 1

    try:
        quat = sensor.game_quaternion or sensor.quaternion
    except Exception:
        quat = None

    try:
        gyro = sensor.gyro
    except Exception:
        gyro = None

    # Read INA219 battery if available
    v_batt = None
    try:
        bus = smbus.SMBus(1)
        raw = bus.read_word_data(0x40, 0x02)
        raw_swapped = ((raw & 0xFF) << 8) | (raw >> 8)
        v_batt = ((raw_swapped >> 3) & 0x1FFF) * 0.004
        bus.close()
    except Exception:
        pass

    now = time.time()
    if now - last_print >= 0.25:  # Print 4 times a second
        last_print = now
        elapsed = now - start_time
        
        if quat:
            qx, qy, qz, qw = quat
            yaw, pitch, roll = quat_to_euler(qx, qy, qz, qw)
            heading_str = f"{yaw:6.1f}°"
            pitch_str = f"{pitch:5.1f}°"
            roll_str = f"{roll:5.1f}°"
        else:
            heading_str = "Waiting.."
            pitch_str = "--"
            roll_str = "--"

        gz_str = f"{gyro[2]:+6.2f} rad/s" if gyro else "--"
        batt_str = f"{v_batt:4.2f} V" if v_batt is not None else "--"

        print(f"{elapsed:6.1f}s | {heading_str:^9} | {pitch_str:^8} | {roll_str:^8} | {gz_str:^10} | {batt_str:^9}")

print("-" * 65)
print("✅ Test complete! If heading changed as you moved the robot, the IMU hardware is functioning perfectly.")
print("=" * 65)
