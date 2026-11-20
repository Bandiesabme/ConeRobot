#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: Raspberry Pi 5 System Info & Network Link Monitor
==============================================================================
Target Platform: Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy

Description:
    Dedicated host-health telemetry node that reads Linux kernel virtual files
    (/proc/stat, /proc/meminfo, /sys/class/thermal, /proc/net/*) directly with
    zero external dependencies (no psutil) and zero subprocesses.

Publishes:
    - `/robot/system_info` (std_msgs/msg/String): JSON payload at 0.5 Hz containing:
        * CPU load (%)
        * RAM & Swap usage (MB, %)
        * SoC temperature (°C)
        * Active network interface (wlusb0 vs wlan0 vs eth0), driver & USB flag
        * Wi-Fi RSSI (dBm), link quality (%), wireless TX retries, RX/TX drops
        * Host uptime (s)
==============================================================================
"""

import json
import os
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemInfoNode(Node):
    """Lightweight ROS 2 node publishing Pi 5 CPU, RAM, Temp, and Wi-Fi link stats."""

    def __init__(self) -> None:
        super().__init__('system_info_node')

        self.declare_parameter('publish_rate_hz', 0.5)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        rate_hz = max(0.1, min(2.0, rate_hz))

        self.pub = self.create_publisher(String, '/robot/system_info', 10)

        # Previous /proc/stat CPU counters for accurate interval utilization
        self._prev_cpu_total: Optional[int] = None
        self._prev_cpu_idle: Optional[int] = None
        self._read_cpu_percent()  # Seed baseline so first published tick is valid

        # Cached per-interface metadata: { iface: (driver, is_usb) }
        self._iface_meta_cache: dict = {}
        # Last valid RSSI per interface to smooth transient rtl8xxxu 0 / -256 ticks
        self._last_valid_rssi: dict = {}

        self.timer = self.create_timer(1.0 / rate_hz, self._publish_system_info)
        self.get_logger().info(
            f"System Info Node started (publishing /robot/system_info at {rate_hz:.2f} Hz)"
        )

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
