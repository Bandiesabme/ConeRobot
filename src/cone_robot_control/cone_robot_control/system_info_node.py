#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: Raspberry Pi 5 System Info, Wi-Fi Link & INA219 Battery Monitor
==============================================================================
Target Platform: Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy

Description:
    Dedicated host-health & power telemetry node:
      - Reads Linux kernel virtual files (/proc/stat, /proc/meminfo,
        /sys/class/thermal, /proc/net/*) directly (no psutil, no subprocesses).
      - Reads INA219 Bus Voltage (Reg 0x02, voltage-only) over Linux kernel
        I2C (smbus) with EMA load-sag smoothing and a realistic 21-point LiPo
        per-cell discharge lookup curve (piecewise-linear interpolation).

Publishes:
    - `/robot/system_info` (std_msgs/msg/String): JSON payload at 0.5 Hz:
        * CPU load (%), RAM & Swap usage (MB, %), SoC temperature (°C), uptime
        * Active network interface (wlusb0 vs wlan0 vs eth0), driver, USB flag,
          Wi-Fi RSSI (dBm), link quality (%), wireless TX retries, RX/TX drops
        * Battery voltage (V) & LiPo State-of-Charge (%) when INA219 present
    - `/battery_state` (sensor_msgs/msg/BatteryState): 1 Hz LiPo status for
        MDD10 voltage compensation, motion controller, and Fleet Dashboard.
==============================================================================
"""

import json
import os
import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

try:
    from .i2c_bus_guard import i2c_bus_lock
except (ImportError, ValueError):
    try:
        from cone_robot_control.i2c_bus_guard import i2c_bus_lock
    except ImportError:
        import i2c_bus_guard
        i2c_bus_lock = i2c_bus_guard.i2c_bus_lock

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


# Realistic LiPo resting open-circuit discharge curve: (cell_voltage_v, soc_percent)
# Piecewise-linear interpolation cuts % error from ~20-30% (linear) down to ~5% at rest.
LIPO_CELL_SOC_LUT = (
    (3.50, 0.0),
    (3.61, 5.0),
    (3.69, 10.0),
    (3.71, 15.0),
    (3.73, 20.0),
    (3.75, 25.0),
    (3.77, 30.0),
    (3.79, 35.0),
    (3.80, 40.0),
    (3.82, 45.0),
    (3.84, 50.0),
    (3.85, 55.0),
    (3.87, 60.0),
    (3.91, 65.0),
    (3.95, 70.0),
    (3.98, 75.0),
    (4.02, 80.0),
    (4.08, 85.0),
    (4.11, 90.0),
    (4.15, 95.0),
    (4.20, 100.0),
)


def lipo_voltage_to_soc_pct(pack_voltage_v: float, cell_count: int = 3) -> float:
    """Converts LiPo pack voltage to State-of-Charge [0.0, 100.0] via 21-point LiPo curve."""
    cells = max(1, int(cell_count))
    v_cell = float(pack_voltage_v) / cells

    if v_cell <= LIPO_CELL_SOC_LUT[0][0]:
        return 0.0
    if v_cell >= LIPO_CELL_SOC_LUT[-1][0]:
        return 100.0

    for i in range(len(LIPO_CELL_SOC_LUT) - 1):
        v0, p0 = LIPO_CELL_SOC_LUT[i]
        v1, p1 = LIPO_CELL_SOC_LUT[i + 1]
        if v0 <= v_cell <= v1:
            t = (v_cell - v0) / max(1e-6, (v1 - v0))
            return p0 + t * (p1 - p0)
    return 0.0


class SystemInfoNode(Node):
    """Dedicated ROS 2 node publishing Pi 5 CPU, RAM, Temp, Wi-Fi, and INA219 LiPo battery stats."""

    def __init__(self) -> None:
        super().__init__('system_info_node')

        # --- System Info Parameters ---
        self.declare_parameter('publish_rate_hz', 0.5)
        rate_hz = max(0.1, min(2.0, float(self.get_parameter('publish_rate_hz').value)))

        # --- INA219 Voltage-Only LiPo Battery Monitor Parameters ---
        self.declare_parameter('enable_battery_monitor', True)
        self.declare_parameter('battery_i2c_bus', 1)
        self.declare_parameter('battery_i2c_address', 64)       # 0x40 default (CJMCU-219 / INA219)
        self.declare_parameter('battery_publish_rate_hz', 1.0)
        self.declare_parameter('battery_cell_count', 3)         # 3S LiPo
        self.declare_parameter('battery_ema_alpha', 0.15)       # ~10-12s EMA smoothing at 1 Hz
        self.declare_parameter('battery_min_v', 10.5)           # 3.50V/cell cutoff (0%)
        self.declare_parameter('battery_max_v', 12.6)           # 4.20V/cell full (100%)
        self.declare_parameter('battery_low_warn_v', 11.2)      # ~20% resting 3S LiPo warning
        self.declare_parameter('battery_frame_id', 'base_link')

        self.enable_battery_monitor = bool(self.get_parameter('enable_battery_monitor').value)
        self.battery_i2c_bus = int(self.get_parameter('battery_i2c_bus').value)
        self.battery_i2c_address = int(self.get_parameter('battery_i2c_address').value)
        self.battery_publish_rate_hz = max(0.1, float(self.get_parameter('battery_publish_rate_hz').value))
        self.battery_cell_count = max(1, int(self.get_parameter('battery_cell_count').value))
        self.battery_ema_alpha = max(0.02, min(1.0, float(self.get_parameter('battery_ema_alpha').value)))
        self.battery_min_v = float(self.get_parameter('battery_min_v').value)
        self.battery_max_v = float(self.get_parameter('battery_max_v').value)
        self.battery_low_warn_v = float(self.get_parameter('battery_low_warn_v').value)
        self.battery_frame_id = str(self.get_parameter('battery_frame_id').value)

        # --- Publishers ---
        self.pub = self.create_publisher(String, '/robot/system_info', 10)
        self.battery_pub = (
            self.create_publisher(BatteryState, '/battery_state', 10)
            if self.enable_battery_monitor
            else None
        )

        # Previous /proc/stat CPU counters for accurate interval utilization
        self._prev_cpu_total: Optional[int] = None
        self._prev_cpu_idle: Optional[int] = None
        self._read_cpu_percent()  # Seed baseline so first published tick is valid

        # Cached per-interface metadata & last valid RSSI (smooths rtl8xxxu 0/-256 ticks)
        self._iface_meta_cache: dict = {}
        self._last_valid_rssi: dict = {}

        # INA219 SMBus handle & smoothed LiPo voltage state
        self.smbus_bus = None
        self._battery_smoothed_v: Optional[float] = None
        self._battery_last_pct: Optional[float] = None
        self._last_battery_warn_time: float = 0.0
        self._battery_err_logged: bool = False

        if self.enable_battery_monitor:
            self._init_battery_monitor()
            self.battery_timer = self.create_timer(
                1.0 / self.battery_publish_rate_hz,
                self._publish_battery_data,
            )

        self.timer = self.create_timer(1.0 / rate_hz, self._publish_system_info)
        self.get_logger().info(
            f"System Info Node started (/robot/system_info @ {rate_hz:.2f} Hz, "
            f"INA219 battery={'ON' if self.enable_battery_monitor else 'OFF'})"
        )

    # --------------------------------------------------------------------------
    # INA219 Voltage-Only LiPo Battery Monitor (Shared Linux Kernel I2C)
    # --------------------------------------------------------------------------
    def _init_battery_monitor(self) -> None:
        """Opens native Linux kernel SMBus (thread/process-safe I2C bus locking)."""
        if not SMBUS_AVAILABLE:
            self.get_logger().warn(
                'smbus/smbus2 not installed — INA219 battery monitoring disabled.'
            )
            return
        try:
            self.smbus_bus = smbus.SMBus(self.battery_i2c_bus)
            self.get_logger().info(
                f"INA219 Battery Monitor ready on /dev/i2c-{self.battery_i2c_bus} "
                f"(0x{self.battery_i2c_address:02X}, {self.battery_cell_count}S LiPo LUT, "
                f"EMA alpha={self.battery_ema_alpha:.2f}, warn<={self.battery_low_warn_v:.1f}V)"
            )
        except Exception as e:
            self.smbus_bus = None
            self.get_logger().warn(f"Could not open /dev/i2c-{self.battery_i2c_bus} for INA219: {e}")

    def _read_ina219_bus_voltage(self) -> Optional[float]:
        """Reads INA219 Reg 0x02 (Bus Voltage, 4 mV LSB, voltage-only) under shared I2C flock."""
        if self.smbus_bus is None:
            return None
        try:
            with i2c_bus_lock(self.battery_i2c_bus):
                raw = self.smbus_bus.read_word_data(self.battery_i2c_address, 0x02)
            # INA219 sends big-endian; SMBus read_word_data returns little-endian
            raw_swapped = ((raw & 0xFF) << 8) | (raw >> 8)
            raw_bits = (raw_swapped >> 3) & 0x1FFF
            voltage_v = raw_bits * 0.004
            self._battery_err_logged = False
            # Reject clearly disconnected / bogus readings (< 1.0 V or > 26.0 V)
            if voltage_v < 1.0 or voltage_v > 26.0:
                return None
            return voltage_v
        except Exception as e:
            if not self._battery_err_logged:
                self._battery_err_logged = True
                self.get_logger().warn(
                    f"INA219 I2C read error at 0x{self.battery_i2c_address:02X}: {e} "
                    "(further errors throttled)"
                )
            return None

    def _publish_battery_data(self) -> None:
        """1 Hz timer reading INA219 voltage, applying EMA smoothing + LiPo LUT, publishing /battery_state."""
        if not self.enable_battery_monitor or self.battery_pub is None or self.smbus_bus is None:
            return

        raw_v = self._read_ina219_bus_voltage()
        if raw_v is None:
            return

        # Exponential Moving Average smooths brief motor load voltage sag without lag on steady drain
        if self._battery_smoothed_v is None:
            self._battery_smoothed_v = raw_v
        else:
            a = self.battery_ema_alpha
            self._battery_smoothed_v = a * raw_v + (1.0 - a) * self._battery_smoothed_v

        v_filt = self._battery_smoothed_v
        soc_pct = lipo_voltage_to_soc_pct(v_filt, self.battery_cell_count)
        self._battery_last_pct = soc_pct

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.battery_frame_id
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.voltage = float(round(v_filt, 2))
        msg.current = float('nan')  # Voltage-only wiring (current not measured)
        msg.percentage = float(round(soc_pct / 100.0, 3))

        if v_filt <= self.battery_min_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif v_filt <= self.battery_low_warn_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.battery_pub.publish(msg)

        now = time.monotonic()
        if v_filt <= self.battery_low_warn_v and (now - self._last_battery_warn_time > 15.0):
            self._last_battery_warn_time = now
            self.get_logger().warn(
                f"⚠️ LOW BATTERY ({self.battery_cell_count}S LiPo): "
                f"{v_filt:.2f}V (~{round(soc_pct)}%). Recharge recommended!"
            )

    # --------------------------------------------------------------------------
    # Pi 5 Host Health & Wi-Fi Link Stats (/proc & /sys, zero subprocesses)
    # --------------------------------------------------------------------------
    def _read_cpu_percent(self) -> Optional[float]:
        """Computes aggregate CPU utilization (%) from /proc/stat counter deltas."""
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
            parts = line.split()
            if not parts or parts[0] != 'cpu' or len(parts) < 5:
                return None
            values = [int(v) for v in parts[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)  # idle + iowait
            total = sum(values)

            prev_total = self._prev_cpu_total
            prev_idle = self._prev_cpu_idle
            self._prev_cpu_total = total
            self._prev_cpu_idle = idle

            if prev_total is None or prev_idle is None:
                return None
            d_total = total - prev_total
            d_idle = idle - prev_idle
            if d_total <= 0:
                return 0.0
            busy_pct = 100.0 * (1.0 - (d_idle / d_total))
            return round(max(0.0, min(100.0, busy_pct)), 1)
        except Exception:
            return None

    def _read_memory_stats(self) -> dict:
        """Reads RAM & Swap usage (MB, %) from /proc/meminfo."""
        out = {
            "ram_used_mb": None,
            "ram_total_mb": None,
            "ram_percent": None,
            "swap_used_mb": None,
            "swap_total_mb": None,
        }
        try:
            info = {}
            with open('/proc/meminfo', 'r') as f:
                for line in f:
                    if ':' not in line:
                        continue
                    key, rest = line.split(':', 1)
                    if key in ('MemTotal', 'MemAvailable', 'MemFree', 'Buffers', 'Cached', 'SwapTotal', 'SwapFree'):
                        parts = rest.split()
                        if parts:
                            info[key] = int(parts[0])  # kB

            mem_total_kb = info.get('MemTotal')
            if mem_total_kb and mem_total_kb > 0:
                mem_avail_kb = info.get('MemAvailable')
                if mem_avail_kb is None:
                    mem_avail_kb = info.get('MemFree', 0) + info.get('Buffers', 0) + info.get('Cached', 0)
                mem_used_kb = max(0, mem_total_kb - mem_avail_kb)
                out["ram_total_mb"] = round(mem_total_kb / 1024.0)
                out["ram_used_mb"] = round(mem_used_kb / 1024.0)
                out["ram_percent"] = round((mem_used_kb / mem_total_kb) * 100.0, 1)

            swap_total_kb = info.get('SwapTotal')
            if swap_total_kb is not None and swap_total_kb >= 0:
                swap_free_kb = info.get('SwapFree', 0)
                swap_used_kb = max(0, swap_total_kb - swap_free_kb)
                out["swap_total_mb"] = round(swap_total_kb / 1024.0)
                out["swap_used_mb"] = round(swap_used_kb / 1024.0)
        except Exception:
            pass
        return out

    def _read_soc_temp_c(self) -> Optional[float]:
        """Reads Raspberry Pi SoC temperature (°C) from thermal_zone0."""
        try:
            path = '/sys/class/thermal/thermal_zone0/temp'
            if os.path.exists(path):
                with open(path, 'r') as f:
                    val = f.read().strip()
                if val:
                    return round(float(val) / 1000.0, 1)
        except Exception:
            pass
        return None

    def _read_uptime_s(self) -> Optional[int]:
        """Reads host uptime in seconds from /proc/uptime."""
        try:
            with open('/proc/uptime', 'r') as f:
                parts = f.read().split()
            if parts:
                return int(float(parts[0]))
        except Exception:
            pass
        return None

    def _get_iface_meta(self, iface: str) -> Tuple[Optional[str], bool]:
        """Returns cached (driver_name, is_usb) for a network interface from sysfs."""
        cached = self._iface_meta_cache.get(iface)
        if cached is not None:
            return cached

        driver = None
        is_usb = False
        try:
            dev_link = f"/sys/class/net/{iface}/device"
            if os.path.exists(dev_link):
                real_dev = os.path.realpath(dev_link)
                is_usb = ('/usb' in real_dev)
                drv_link = os.path.join(dev_link, 'driver')
                if os.path.exists(drv_link):
                    driver = os.path.basename(os.path.realpath(drv_link))
        except Exception:
            pass

        meta = (driver, is_usb)
        if driver is not None:
            self._iface_meta_cache[iface] = meta
        return meta

    def _read_wifi_stats(self) -> dict:
        """
        Reads active default-route interface & Wi-Fi RSSI/quality/drops directly
        from Linux /proc/net/* virtual files (zero subprocesses).
        """
        stats = {
            "iface": None,
            "driver": None,
            "is_usb": None,
            "rssi_dbm": None,
            "quality_pct": None,
            "retries": None,
            "rx_drop": None,
            "tx_drop": None,
        }

        # 1. Active default-route interface (lowest metric wins during failover)
        default_iface = None
        best_metric = None
        try:
            with open('/proc/net/route', 'r') as f:
                next(f, None)
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 8 and parts[1] == '00000000' and parts[7] == '00000000':
                        flags = int(parts[3], 16)
                        if flags & 1:
                            metric = int(parts[6])
                            if best_metric is None or metric < best_metric:
                                best_metric = metric
                                default_iface = parts[0]
        except Exception:
            pass

        # 2. Wireless signal level (dBm), link quality, and TX retries
        wireless_entries = {}
        try:
            with open('/proc/net/wireless', 'r') as f:
                lines = f.readlines()
            for line in lines[2:]:
                if ':' not in line:
                    continue
                ifname, rest = line.split(':', 1)
                ifname = ifname.strip()
                fields = rest.split()
                if len(fields) >= 3:
                    qual_raw = float(fields[1].rstrip('.'))
                    level_raw = float(fields[2].rstrip('.'))
                    retries = int(float(fields[7].rstrip('.'))) if len(fields) >= 8 else None

                    rssi_dbm = None
                    now_ts = time.monotonic()
                    if -130.0 < level_raw < 0.0:
                        rssi_dbm = round(level_raw, 1)
                        self._last_valid_rssi[ifname] = (rssi_dbm, now_ts)
                    elif 128.0 < level_raw <= 255.0:
                        rssi_dbm = round(level_raw - 256.0, 1)
                        self._last_valid_rssi[ifname] = (rssi_dbm, now_ts)
                    elif qual_raw > 0.0:
                        # Realtek rtl8xxxu occasionally outputs 0 or -256 at very close range
                        # (100% quality) or between RX beacons. Hold recent RSSI or invert
                        # Linux cfg80211's qual = clamp(signal, -110, -40) + 110 formula.
                        cached_rssi = self._last_valid_rssi.get(ifname)
                        if qual_raw >= 70.0:
                            rssi_dbm = -40.0
                        elif cached_rssi is not None and (now_ts - cached_rssi[1]) <= 6.0:
                            rssi_dbm = cached_rssi[0]
                        elif qual_raw <= 70.0:
                            rssi_dbm = round(qual_raw - 110.0, 1)
                        else:
                            rssi_dbm = round(-100.0 + (min(100.0, qual_raw) * 0.6), 1)

                    quality_pct = None
                    if qual_raw > 0.0:
                        if qual_raw <= 70.0:
                            quality_pct = int(round((qual_raw / 70.0) * 100.0))
                        else:
                            quality_pct = int(min(100, round(qual_raw)))
                    elif rssi_dbm is not None:
                        quality_pct = int(round(max(0.0, min(100.0, (rssi_dbm + 100.0) * 2.0))))

                    wireless_entries[ifname] = (rssi_dbm, quality_pct, retries)
        except Exception:
            pass

        chosen_iface = default_iface
        if chosen_iface is None and wireless_entries:
            chosen_iface = next(iter(wireless_entries.keys()))

        stats["iface"] = chosen_iface
        if chosen_iface and chosen_iface in wireless_entries:
            rssi_dbm, quality_pct, retries = wireless_entries[chosen_iface]
            stats["rssi_dbm"] = rssi_dbm
            stats["quality_pct"] = quality_pct
            stats["retries"] = retries

        if chosen_iface:
            stats["driver"], stats["is_usb"] = self._get_iface_meta(chosen_iface)

            # 3. RX / TX packet drop counters from /proc/net/dev
            try:
                with open('/proc/net/dev', 'r') as f:
                    for line in f:
                        if ':' not in line:
                            continue
                        ifname, rest = line.split(':', 1)
                        if ifname.strip() == chosen_iface:
                            cols = rest.split()
                            if len(cols) >= 12:
                                stats["rx_drop"] = int(cols[3])
                                stats["tx_drop"] = int(cols[11])
                            break
            except Exception:
                pass

        return stats

    def _publish_system_info(self) -> None:
        try:
            cpu_pct = self._read_cpu_percent()
            mem = self._read_memory_stats()
            temp_c = self._read_soc_temp_c()
            wifi = self._read_wifi_stats()
            uptime_s = self._read_uptime_s()

            payload = {
                "cpu_percent": cpu_pct,
                "ram_used_mb": mem["ram_used_mb"],
                "ram_total_mb": mem["ram_total_mb"],
                "ram_percent": mem["ram_percent"],
                "swap_used_mb": mem["swap_used_mb"],
                "swap_total_mb": mem["swap_total_mb"],
                "cpu_temp_c": temp_c,
                "cpu_temp": f"{temp_c:.1f} °C" if temp_c is not None else "N/A",
                "uptime_s": uptime_s,
                "battery_v": round(self._battery_smoothed_v, 2) if self._battery_smoothed_v is not None else None,
                "battery_pct": round(self._battery_last_pct, 1) if self._battery_last_pct is not None else None,
                "wifi_iface": wifi["iface"],
                "wifi_driver": wifi["driver"],
                "wifi_is_usb": wifi["is_usb"],
                "wifi_rssi_dbm": wifi["rssi_dbm"],
                "wifi_quality_pct": wifi["quality_pct"],
                "wifi_retries": wifi["retries"],
                "wifi_rx_drop": wifi["rx_drop"],
                "wifi_tx_drop": wifi["tx_drop"],
            }

            msg = String()
            msg.data = json.dumps(payload)
            self.pub.publish(msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        if self.smbus_bus is not None:
            try:
                self.smbus_bus.close()
            except Exception:
                pass
            self.smbus_bus = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemInfoNode()
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
