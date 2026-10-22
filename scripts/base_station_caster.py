#!/usr/bin/env python3
"""
==============================================================================
Raspberry Pi RTK Base Station NTRIP Caster & Live Web Dashboard
==============================================================================
Description:
    Pure Standalone Python 3 application (Zero ROS dependencies).
    Reads raw RTCM3 differential correction packets and GNSS position from
    the Base GNSS HAT (Waveshare LC29H(BS) / LC29H(EA) on /dev/ttyAMA0 @ 115200)
    and provides:
      1. Local NTRIP 1.0/2.0 Caster TCP server on port 2101 (for rovers).
      2. Non-blocking high-speed async TCP multicast engine (zero serial delay).
      3. Automatic 1-Hour Survey-In Calibration with Auto-Lock & Persistence.
      4. Instant reload of frozen static coordinates on future boots (0 mm drift).
      5. Multi-threaded, lightning-fast HTTP Web Dashboard on port 8080.
      6. Web Dashboard UI with "Lock Now" and "Recalibrate" buttons.
      7. 100% offline-ready (zero external CDN or font dependencies).

Usage:
    python3 base_station_caster.py --port 2101 --web-port 8080 --mountpoint BASE --survey-time 3600
==============================================================================
"""

import argparse
from collections import deque
from datetime import datetime
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import select
import socket
import sys
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple


def _build_crc24q_table() -> List[int]:
    """Precomputes the CRC-24Q lookup table used by RTCM3 framing (poly 0x1864CFB)."""
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC24Q_TABLE = _build_crc24q_table()


def crc24q(data: bytes) -> int:
    """Computes RTCM3 CRC-24Q over header + payload."""
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[((crc >> 16) ^ b) & 0xFF]
    return crc


class NTRIPBaseCaster:
    def __init__(
        self,
        serial_port: str = "/dev/ttyAMA0",
        baud_rate: int = 115200,
        server_port: int = 2101,
        web_port: int = 8080,
        mountpoint: str = "BASE",
        password: str = "none",
        survey_duration: int = 3600,
        survey_accuracy: float = 0.5,
        recalibrate: bool = False,
        use_saved: bool = False,
        fixed_lat: Optional[float] = None,
        fixed_lon: Optional[float] = None,
        fixed_alt: Optional[float] = None
    ) -> None:
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.server_port = server_port
        self.web_port = web_port
        self.mountpoint = mountpoint.strip("/")
        self.password = password

        # Mapping: client_socket -> (ip, port, connect_time, bytes_sent)
        self.clients_map: Dict[socket.socket, dict] = {}
        self.clients_lock = threading.Lock()
        self.is_running = True
        self.start_time = time.time()
        self.total_bytes_sent = 0
        self.total_rtcm_bytes_read = 0
        self.rtcm_packet_count = 0

        # Log history buffer
        self.logs = deque(maxlen=100)
        self.logs_lock = threading.Lock()

        # Calibration & Auto-Lock State
        self.survey_target_duration = survey_duration
        self.survey_target_accuracy = survey_accuracy
        self.survey_start_time: Optional[float] = None
        self.survey_duration = 0
        self.survey_status = "INITIALIZING"
        self.survey_valid = False
        self.survey_accuracy = 99.9
        self.is_static_fixed = False
        self.locked_timestamp = ""

        # Persistence file paths
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "base_station_fixed_coords.json")
        self.locations_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "base_station_locations.json")
        self.active_location: Optional[str] = None

        # Coordinate sample history (for averaging)
        self.coord_samples: List[Tuple[float, float, float]] = []
        self.survey_lat = 0.0
        self.survey_lon = 0.0
        self.survey_alt = 0.0
        self.satellites_tracked = 0
        self.msm_sats: Dict[int, int] = {}
        self.hdop = 1.0
        self.local_ip = self._get_local_ip()
        self.ser = None
        self.ser_lock = threading.Lock()
        self.rtcm_1005_count = 0
        self.last_serial_time: Optional[float] = None
        self.rtcm_crc_errors = 0
        self._last_gga_time = 0.0

        # RTCM 1005 verification (what the hardware actually broadcasts vs. what we locked)
        self._locked_ecef: Optional[Tuple[float, float, float]] = None
        self._last_1005_ecef: Optional[Tuple[float, float, float]] = None
        self.base_1005_offset_m: Optional[float] = None
        self._last_1005_warn_time = 0.0

        # Hardware Button Input (GPIO 17 / D17 to GND with Internal Pull-Up)
        self.button_pin = 17
        self.button_dev = None
        self.button_backend = None
        self._button_press_start: Optional[float] = None
        self._button_held_sec: float = 0.0
        self._shutdown_initiated: bool = False
        self._init_button()

        # Battery Sensor Smoothing & Moving Average (~3 second sliding window)
        self._batt_v_history: Deque[float] = deque(maxlen=8)
        self._batt_ma_history: Deque[float] = deque(maxlen=8)
        self._ina219_configured = False

        # GNSS Satellite Signal SNR (C/N0 in dB-Hz) Tracking
        self.sat_snr_map: Dict[str, dict] = {}
        self.sat_snr_lock = threading.Lock()
        self._last_gsv_time = 0.0

        # Auto-restore saved coordinates by default on boot (prevents recalibration after power drops)
        if fixed_lat is not None and fixed_lon is not None:
            self._apply_fixed_coords(fixed_lat, fixed_lon, fixed_alt or 0.0, "Command-Line Arguments")
        elif not recalibrate:
            store = self._load_locations_store()
            act = store.get("active_location")
            if act and act in store.get("locations", {}):
                self.load_named_location(act)
                self._add_log(f"🚀 Auto-loaded active preset '{act}'. Base station ready with 0 mm drift!")
            elif self._load_saved_coords():
                self._add_log("🚀 Auto-loaded saved static coordinates from previous session. Base station ready with 0 mm drift!")
            else:
                self._add_log("⏳ No saved coordinates found (or recalibrate requested). Starting Survey-In calibration...")
        else:
            self._add_log("🔄 Recalibrate requested. Starting fresh Survey-In calibration...")

        self._add_log(f"Base Station initialized. Serial: {self.serial_port}, Web: http://{self.local_ip}:{self.web_port}")

    @staticmethod
    def _atomic_write_json(path: str, data: dict) -> None:
        """Power-loss-safe JSON write: temp file + fsync + atomic rename."""
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(data, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
        try:
            dir_fd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
        except Exception:
            pass

    def _load_locations_store(self) -> dict:
        """Loads all named location profiles from JSON with migration from legacy format."""
        if os.path.exists(self.locations_file):
            try:
                with open(self.locations_file, "r") as f:
                    data = json.load(f)
                if isinstance(data, dict) and "locations" in data:
                    return data
            except Exception:
                pass

        # Migration from legacy single-config file if exists
        store = {"active_location": None, "locations": {}}
        legacy = self._get_saved_coords_info()
        if legacy:
            name = "Default Benchmark"
            store["active_location"] = name
            store["locations"][name] = {
                "lat": legacy["lat"],
                "lon": legacy["lon"],
                "alt": legacy.get("alt", 0.0),
                "timestamp": legacy.get("timestamp", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            }
            self._save_locations_store(store)
        return store

    def _save_locations_store(self, store: dict) -> None:
        """Persists location profiles to disk."""
        try:
            self._atomic_write_json(self.locations_file, store)
        except Exception as e:
            self._add_log(f"Failed to save locations: {e}", "ERROR")

    def save_named_location(self, name: str) -> bool:
        """Saves current surveyed or fixed coordinates under a custom preset name."""
        if not name or not name.strip():
            return False
        name = name.strip()

        lat, lon, alt = self.survey_lat, self.survey_lon, self.survey_alt
        if (lat == 0.0 or lon == 0.0) and self.coord_samples:
            lats = [s[0] for s in self.coord_samples]
            lons = [s[1] for s in self.coord_samples]
            alts = [s[2] for s in self.coord_samples]
            lat = sum(lats) / len(lats)
            lon = sum(lons) / len(lons)
            alt = sum(alts) / len(alts)

        if lat == 0.0 or lon == 0.0:
            self._add_log("Cannot save preset: No valid GNSS coordinates available yet!", "WARN")
            return False

        store = self._load_locations_store()
        store["locations"][name] = {
            "lat": round(lat, 8),
            "lon": round(lon, 8),
            "alt": round(alt, 2),
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        store["active_location"] = name
        self.active_location = name
        self._save_locations_store(store)

        # Also write legacy single config for backward compatibility
        try:
            self._atomic_write_json(self.config_file, {
                "is_locked": True,
                "lat": round(lat, 8),
                "lon": round(lon, 8),
                "alt": round(alt, 2),
                "samples": len(self.coord_samples) or 100,
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            })
        except Exception:
            pass

        self._apply_fixed_coords(lat, lon, alt, f"Preset '{name}'")
        self._add_log(f"💾 Saved & locked location preset '{name}': ({lat:.8f}°, {lon:.8f}°, {alt:.2f}m)")
        return True

    def load_named_location(self, name: str) -> bool:
        """Applies a previously saved location profile by name with 0 mm drift."""
        store = self._load_locations_store()
        loc = store.get("locations", {}).get(name)
        if not loc:
            self._add_log(f"Preset '{name}' not found!", "WARN")
            return False

        lat = float(loc["lat"])
        lon = float(loc["lon"])
        alt = float(loc.get("alt", 0.0))
        self._apply_fixed_coords(lat, lon, alt, f"Preset '{name}'")
        self.active_location = name
        store["active_location"] = name
        self._save_locations_store(store)
        self._add_log(f"🎯 Switched to preset '{name}': ({lat:.8f}°, {lon:.8f}°, {alt:.2f}m) [0 mm Drift]")
        return True

    def delete_named_location(self, name: str) -> bool:
        """Deletes a saved location profile by name."""
        store = self._load_locations_store()
        if name in store.get("locations", {}):
            del store["locations"][name]
            if store.get("active_location") == name:
                store["active_location"] = None
                self.active_location = None
            self._save_locations_store(store)
            self._add_log(f"🗑️ Deleted location preset '{name}'")
            return True
        return False

    def _get_saved_coords_info(self) -> Optional[dict]:
        """Returns saved coordinates dictionary if present on disk."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)
                if data.get("is_locked"):
                    return data
            except Exception:
                pass
        return None

    def use_saved_now(self) -> bool:
        """Explicitly applies previously saved static coordinates upon user request."""
        return self._load_saved_coords()

    def _add_log(self, text: str, level: str = "INFO") -> None:
        """Adds a log entry with timestamp for the web console and terminal."""
        ts = datetime.now().strftime("%H:%M:%S")
        entry = {"time": ts, "level": level, "msg": text}
        with self.logs_lock:
            self.logs.append(entry)

    def _get_local_ip(self) -> str:
        """Helper to get primary network IP address across any router, hotspot, or offline network."""
        for target in ["8.8.8.8", "192.168.0.1", "192.168.1.1", "192.168.137.1", "10.0.0.1", "1.1.1.1"]:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect((target, 80))
                ip = s.getsockname()[0]
                s.close()
                if ip and not ip.startswith("127."):
                    return ip
            except Exception:
                pass
        try:
            import subprocess
            out = subprocess.check_output(["hostname", "-I"], timeout=1.0).decode('ascii').strip()
            ips = [i for i in out.split() if not i.startswith("127.")]
            if ips:
                return ips[0]
        except Exception:
            pass
        try:
            host_ip = socket.gethostbyname(socket.gethostname())
            if host_ip and not host_ip.startswith("127."):
                return host_ip
        except Exception:
            pass
        return "127.0.0.1"

    def _udp_beacon_loop(self) -> None:
        """Broadcasts a periodic UDP discovery beacon every 1 second on port 2102 for zero-config rover pairing."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        beacon_port = 2102

        while self.is_running:
            try:
                current_ip = self._get_local_ip()
                if current_ip and not current_ip.startswith("127."):
                    payload = json.dumps({
                        "service": "conerobot-rtk-base",
                        "ip": current_ip,
                        "port": self.server_port,
                        "web_port": self.web_port,
                        "mountpoint": self.mountpoint,
                        "status": "LOCKED" if self.is_static_fixed else "CALIBRATING",
                        "sats": self.satellites_tracked
                    }).encode('utf-8')
                    sock.sendto(payload, ("255.255.255.255", beacon_port))
            except Exception:
                pass
            time.sleep(1.0)

    def _load_saved_coords(self) -> bool:
        """Loads previously locked static base coordinates if available."""
        saved = self._get_saved_coords_info()
        if saved:
            try:
                lat = float(saved["lat"])
                lon = float(saved["lon"])
                alt = float(saved.get("alt", 0.0))
                self._apply_fixed_coords(lat, lon, alt, f"Saved Config ({saved.get('timestamp', 'Unknown')})")
                return True
            except Exception as e:
                self._add_log(f"Failed to read saved coords: {e}", "WARN")
        return False

    @staticmethod
    def _nmea_checksum(sentence: str) -> str:
        """Calculates 2-digit uppercase hex NMEA checksum."""
        clean = sentence.strip().lstrip('$').split('*')[0]
        cs = 0
        for char in clean:
            cs ^= ord(char)
        return f"{cs:02X}"

    @staticmethod
    def _lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        """Converts WGS-84 LLA coordinates to ECEF coordinates (meters)."""
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        n = a / math.sqrt(1.0 - e2 * (math.sin(lat) ** 2))
        x = (n + alt_m) * math.cos(lat) * math.cos(lon)
        y = (n + alt_m) * math.cos(lat) * math.sin(lon)
        z = (n * (1.0 - e2) + alt_m) * math.sin(lat)
        return x, y, z

    def _send_gnss_cmd(self, cmd_body: str) -> None:
        """Sends an NMEA / PAIR / PQTM command to the base station GNSS module."""
        cs = self._nmea_checksum(cmd_body)
        clean = cmd_body.strip().lstrip('$').split('*')[0]
        sentence = f"${clean}*{cs}\r\n".encode('ascii')
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(sentence)
                    self.ser.flush()
                    self._add_log(f"GNSS Command Sent: ${clean}*{cs}")
                except Exception as e:
                    self._add_log(f"Failed to write GNSS command: {e}", "WARN")

    def _init_base_gnss_hardware(self) -> None:
        """Initializes the Waveshare LC29H(BS) HAT with proper RTK Base mode & RTCM outputs."""
        self._add_log("🔧 Configuring LC29H(BS) Hardware: Enabling Base Mode & RTCM 1005/MSM7...")
        # 1. Set receiver mode to Base Station (2)
        self._send_gnss_cmd("PQTMCFGRCVRMODE,W,2")
        time.sleep(0.05)
        # 2. Enable RTCM 1005 (Station Coordinates at 1 Hz)
        self._send_gnss_cmd("PAIR434,1")
        time.sleep(0.05)
        # 3. Enable RTCM MSM7 multi-constellation observations
        self._send_gnss_cmd("PAIR432,1")
        time.sleep(0.05)
        # 4. Enable NMEA GGA sentence output (1 Hz) for live telemetry
        self._send_gnss_cmd("PAIR062,0,1")
        time.sleep(0.05)
        # 5. Enable NMEA GSV sentence output (1 Hz) for satellite C/N0 SNR telemetry
        self._send_gnss_cmd("PAIR062,3,1")
        time.sleep(0.05)
        # 4. If static coordinates are locked, send fixed position to hardware
        if self.is_static_fixed and self.survey_lat != 0.0 and self.survey_lon != 0.0:
            self._send_hardware_fixed_coords(self.survey_lat, self.survey_lon, self.survey_alt)
        else:
            # Start Survey-In in hardware (300s target, 1.5m accuracy)
            self._send_gnss_cmd("PQTMCFGSVIN,W,1,300,1.5,0,0,0")
        time.sleep(0.05)
        self._send_gnss_cmd("PQTMSAVEPAR")

    def _send_hardware_fixed_coords(self, lat: float, lon: float, alt: float) -> None:
        """Sends fixed ECEF coordinates to LC29H(BS) HAT to broadcast in RTCM Message 1005."""
        if lat == 0.0 or lon == 0.0:
            return
        x, y, z = self._lla_to_ecef(lat, lon, alt)
        self._locked_ecef = (x, y, z)
        self._send_gnss_cmd(f"PQTMCFGSVIN,W,2,0,0,{x:.4f},{y:.4f},{z:.4f}")
        time.sleep(0.05)
        self._send_gnss_cmd("PQTMSAVEPAR")
        self._add_log(f"🎯 LC29H Hardware Locked to ({lat:.8f}°, {lon:.8f}°, {alt:.2f}m) [ECEF: {x:.2f}, {y:.2f}, {z:.2f}]")

    def _apply_fixed_coords(self, lat: float, lon: float, alt: float, source: str) -> None:
        """Locks base station into permanent static fixed mode (0 mm drift)."""
        self.survey_lat = lat
        self.survey_lon = lon
        self.survey_alt = alt
        self.is_static_fixed = True
        self.survey_valid = True
        self.survey_status = "LOCKED_STATIC"
        self.survey_accuracy = 0.00
        self.locked_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._add_log(f"🎯 LOCKED STATIC BASE ({source}): ({lat:.8f}°, {lon:.8f}°, {alt:.2f}m) [0 mm Drift]")
        self._send_hardware_fixed_coords(lat, lon, alt)

    def lock_now(self) -> bool:
        """Immediately locks current accumulated average position and writes to disk."""
        if not self.coord_samples:
            if self.survey_lat != 0.0 and self.survey_lon != 0.0:
                mean_lat, mean_lon, mean_alt = self.survey_lat, self.survey_lon, self.survey_alt
            else:
                self._add_log("Cannot lock: No GPS coordinates collected yet!", "WARN")
                return False
        else:
            # Robust median / trimmed average
            lats = sorted([s[0] for s in self.coord_samples])
            lons = sorted([s[1] for s in self.coord_samples])
            alts = sorted([s[2] for s in self.coord_samples])
            trim = max(1, int(len(lats) * 0.05)) if len(lats) > 20 else 0
            if trim > 0:
                lats = lats[trim:-trim]
                lons = lons[trim:-trim]
                alts = alts[trim:-trim]
            mean_lat = sum(lats) / len(lats)
            mean_lon = sum(lons) / len(lons)
            mean_alt = sum(alts) / len(alts)

        self._apply_fixed_coords(mean_lat, mean_lon, mean_alt, "Manual Lock")

        # Save to disk
        try:
            self._atomic_write_json(self.config_file, {
                "is_locked": True,
                "lat": round(mean_lat, 8),
                "lon": round(mean_lon, 8),
                "alt": round(mean_alt, 2),
                "samples": len(self.coord_samples),
                "timestamp": self.locked_timestamp
            })
            self._add_log(f"💾 Saved permanent static coordinates to {os.path.basename(self.config_file)}")

            # Also update locations store active position
            store = self._load_locations_store()
            store["active_location"] = "Last Surveyed Position"
            store["locations"]["Last Surveyed Position"] = {
                "lat": round(mean_lat, 8),
                "lon": round(mean_lon, 8),
                "alt": round(mean_alt, 2),
                "timestamp": self.locked_timestamp
            }
            self._save_locations_store(store)
            self.active_location = "Last Surveyed Position"
            return True
        except Exception as e:
            self._add_log(f"Failed to save coordinates: {e}", "ERROR")
            return False

    def recalibrate(self) -> bool:
        """Clears saved fixed position and restarts a fresh 1-hour calibration survey."""
        try:
            if os.path.exists(self.config_file):
                os.remove(self.config_file)
            store = self._load_locations_store()
            store["active_location"] = None
            self._save_locations_store(store)
            self.active_location = None
        except Exception:
            pass

        self.is_static_fixed = False
        self._locked_ecef = None
        self.base_1005_offset_m = None
        self.survey_valid = False
        self.survey_status = "CALIBRATING"
        self.survey_start_time = None
        self.survey_duration = 0
        self.survey_accuracy = 99.9
        self.coord_samples.clear()
        self._add_log("🔄 Recalibration triggered! Starting fresh Survey-In calibration...")
        return True

    def start(self) -> None:
        """Starts NTRIP Caster, Web Dashboard, and Serial Reader threads."""
        print("=" * 75)
        print("  📡 RASPBERRY PI RTK BASE STATION & WEB DASHBOARD")
        print("=" * 75)
        print(f"  • Base Station IP   : {self.local_ip}")
        print(f"  • Serial Port       : {self.serial_port} @ {self.baud_rate} baud")
        print(f"  • NTRIP Server Port : {self.server_port} (Mountpoint: /{self.mountpoint})")
        print(f"  • 🌐 Web Dashboard  : http://{self.local_ip}:{self.web_port}")
        batt = self._get_battery_data()
        if batt is not None:
            v = batt["voltage"]
            ma = batt["current_ma"]
            print(f"  • 🔋 Battery Monitor: {v:.2f} V | {ma:.1f} mA (INA219 @ 0x40)")
            self._add_log(f"🔋 INA219 Battery Monitor active: {v:.2f} V, {ma:.1f} mA")
        if self.button_backend:
            print(f"  • 🔘 Button Monitor : Active on GPIO {self.button_pin} ({self.button_backend})")
            self._add_log(f"🔘 Hardware button monitor active on GPIO {self.button_pin} ({self.button_backend})")
        if self.is_static_fixed:
            print(f"  • Mode              : 🎯 STATIC FIXED BASE (0 mm Drift)")
            print(f"  • Coordinates       : {self.survey_lat:.8f}°, {self.survey_lon:.8f}°, {self.survey_alt:.2f}m")
        else:
            print(f"  • Mode              : ⏳ Auto-Calibrating ({self.survey_target_duration}s Target)")
        print("=" * 75 + "\n")

        # 1. Start background TCP Server thread for NTRIP Rovers
        ntrip_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        ntrip_thread.start()

        # 2. Start background multi-threaded Web Dashboard HTTP server
        web_thread = threading.Thread(target=self._web_server_loop, daemon=True)
        web_thread.start()

        # 3. Start periodic console logger & survey estimator
        diag_thread = threading.Thread(target=self._diagnostic_logger_loop, daemon=True)
        diag_thread.start()

        # 4. Start background UDP discovery beacon broadcaster (for zero-config auto-discovery)
        beacon_thread = threading.Thread(target=self._udp_beacon_loop, daemon=True)
        beacon_thread.start()

        # 5. Start background hardware button monitor (tracks press & triggers safe shutdown on 3s hold)
        if self.button_backend:
            btn_thread = threading.Thread(target=self._button_monitor_loop, daemon=True)
            btn_thread.start()

        # 6. Run serial reader in main thread
        self._serial_reader_loop()

    def _tcp_server_loop(self) -> None:
        """Listens for incoming NTRIP rover client TCP connections."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server_sock.bind(('0.0.0.0', self.server_port))
            server_sock.listen(10)
            self._add_log(f"NTRIP Server listening on port {self.server_port}")

            while self.is_running:
                client_sock, client_addr = server_sock.accept()
                client_thread = threading.Thread(
                    target=self._handle_client_handshake,
                    args=(client_sock, client_addr),
                    daemon=True
                )
                client_thread.start()
        except Exception as e:
            self._add_log(f"Server error: {e}", "ERROR")

    def _handle_client_handshake(self, client_sock: socket.socket, client_addr: tuple) -> None:
        """Handles NTRIP 1.0 / 2.0 HTTP GET request handshake."""
        ip, port = client_addr
        try:
            client_sock.settimeout(5.0)
            raw_req = client_sock.recv(2048).decode('ascii', errors='ignore')

            if not raw_req:
                client_sock.close()
                return

            lines = raw_req.split('\r\n')
            first_line = lines[0] if lines else ""
            self._add_log(f"Rover handshake from {ip}:{port}: {first_line}")

            # Send standard NTRIP Caster response
            response = (
                "ICY 200 OK\r\n"
                "Server: ConeRobot-RPi5-NTRIPCaster/2.0\r\n"
                "Content-Type: gnss/data\r\n"
                "Connection: close\r\n"
                "\r\n"
            )
            client_sock.sendall(response.encode('ascii'))
            client_sock.settimeout(5.0)
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            meta = {
                "ip": ip,
                "port": port,
                "connected_at": time.time(),
                "bytes_sent": 0,
                "queue": queue.Queue(maxsize=50),
                "alive": True,
                "dropped": 0
            }

            # Deduplicate & register
            with self.clients_lock:
                to_remove = [s for s, m in self.clients_map.items() if m["ip"] == ip]
                for sock in to_remove:
                    self.clients_map[sock]["alive"] = False
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    try:
                        sock.close()
                    except Exception:
                        pass
                    del self.clients_map[sock]

                self.clients_map[client_sock] = meta

            self._add_log(f"Stream Active: Broadcasting RTCM3 to Rover {ip}")

        except Exception as e:
            self._add_log(f"Client handshake error ({ip}): {e}", "WARN")
            try:
                client_sock.close()
            except Exception:
                pass
            return

        # Dedicated per-rover sender: a slow/stalled Wi-Fi link never blocks the serial reader
        self._client_sender_loop(client_sock, meta)

    def _client_sender_loop(self, client_sock: socket.socket, meta: dict) -> None:
        """Drains this rover's RTCM queue onto its socket until it disconnects."""
        q = meta["queue"]
        while self.is_running and meta.get("alive", False):
            try:
                data = q.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                client_sock.sendall(data)
                meta["bytes_sent"] += len(data)
                self.total_bytes_sent += len(data)
            except OSError as e:
                if meta.get("alive", False):
                    self._add_log(f"Rover {meta['ip']} disconnected: {e}", "WARN")
                break

        meta["alive"] = False
        with self.clients_lock:
            if self.clients_map.get(client_sock) is meta:
                del self.clients_map[client_sock]
        try:
            client_sock.close()
        except Exception:
            pass

    def _parse_nmea_coordinate(self, raw_coord: str, direction: str, is_lon: bool = False) -> Optional[float]:
        """Convert NMEA DDMM.MMMM or DDDMM.MMMM format to decimal degrees."""
        if not raw_coord or not direction:
            return None
        try:
            deg_digits = 3 if is_lon else 2
            degrees = float(raw_coord[:deg_digits])
            minutes = float(raw_coord[deg_digits:])
            decimal = degrees + (minutes / 60.0)
            if direction in ['S', 'W']:
                decimal = -decimal
            return decimal
        except ValueError:
            return None

    def _update_survey_statistics(self, lat: float, lon: float, alt: float) -> None:
        """Calculates live Survey-In elapsed duration and position standard deviation."""
        if self.is_static_fixed:
            return

        # Ignore invalid zero coordinates
        if abs(lat) < 1.0 or abs(lon) < 1.0:
            return

        now = time.time()
        if self.survey_start_time is None:
            self.survey_start_time = now
            self._add_log(f"🛰️ GPS Lock acquired! Starting Auto-Calibration timer (Target: {self.survey_target_duration}s)...")

        self.survey_duration = int(now - self.survey_start_time)
        self.coord_samples.append((lat, lon, alt))

        # Keep a rolling window of recent samples (last 300 samples)
        # to ensure old cold-start / satellite acquisition jumps don't corrupt accuracy
        if len(self.coord_samples) > 500:
            self.coord_samples = self.coord_samples[-300:]

        if len(self.coord_samples) >= 5:
            lats = [s[0] for s in self.coord_samples]
            lons = [s[1] for s in self.coord_samples]

            # Use median to filter out wild initial startup outliers (> 30 meters from median)
            median_lat = sorted(lats)[len(lats) // 2]
            median_lon = sorted(lons)[len(lons) // 2]

            lat_m = 111132.0
            lon_m = 111412.0 * math.cos(math.radians(median_lat))

            valid_samples = [
                s for s in self.coord_samples
                if math.sqrt(((s[1] - median_lon) * lon_m)**2 + ((s[0] - median_lat) * lat_m)**2) < 30.0
            ]

            if len(valid_samples) >= 5:
                v_lats = [s[0] for s in valid_samples]
                v_lons = [s[1] for s in valid_samples]
                mean_lat = sum(v_lats) / len(v_lats)
                mean_lon = sum(v_lons) / len(v_lons)

                dx = [(ln - mean_lon) * lon_m for ln in v_lons]
                dy = [(lt - mean_lat) * lat_m for lt in v_lats]
                sigma_2d = math.sqrt((sum(x**2 for x in dx) + sum(y**2 for y in dy)) / len(dx))
                self.survey_accuracy = round(sigma_2d, 2)
                self.survey_lat = mean_lat
                self.survey_lon = mean_lon
                self.survey_alt = sum(s[2] for s in valid_samples) / len(valid_samples)

                # Auto-Lock condition reached
                if (self.survey_duration >= self.survey_target_duration and self.survey_accuracy <= self.survey_target_accuracy) or (self.survey_duration >= self.survey_target_duration * 1.5):
                    self._add_log(f"🎯 Auto-Calibration COMPLETE! (Duration: {self.survey_duration}s, Acc: {self.survey_accuracy:.2f}m)")
                    self.lock_now()
                else:
                    self.survey_status = "CALIBRATING"

    @staticmethod
    def _ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Converts WGS-84 ECEF coordinates (meters) to Latitude, Longitude, and Altitude."""
        a = 6378137.0
        f = 1.0 / 298.257223563
        b = a * (1.0 - f)
        e2 = 1.0 - (b * b) / (a * a)
        e_prime2 = (a * a - b * b) / (b * b)

        p = math.sqrt(x * x + y * y)
        if p < 1e-6:
            lat = 90.0 if z > 0 else -90.0
            return lat, 0.0, abs(z) - b

        theta = math.atan2(z * a, p * b)
        lat = math.atan2(
            z + e_prime2 * b * (math.sin(theta) ** 3),
            p - e2 * a * (math.cos(theta) ** 3)
        )
        lon = math.atan2(y, x)
        n = a / math.sqrt(1.0 - e2 * (math.sin(lat) ** 2))
        alt = p / math.cos(lat) - n
        return math.degrees(lat), math.degrees(lon), alt

    @staticmethod
    def _get_bits(data: bytes, bit_offset: int, num_bits: int) -> int:
        """Extracts an arbitrary unsigned bitfield from a byte sequence."""
        val = 0
        for i in range(num_bits):
            pos = bit_offset + i
            byte_idx = pos // 8
            if byte_idx >= len(data):
                break
            bit_idx = 7 - (pos % 8)
            bit = (data[byte_idx] >> bit_idx) & 1
            val = (val << 1) | bit
        return val

    @staticmethod
    def _get_signed_bits(data: bytes, bit_offset: int, num_bits: int) -> int:
        """Extracts a signed two's-complement bitfield."""
        val = NTRIPBaseCaster._get_bits(data, bit_offset, num_bits)
        if val & (1 << (num_bits - 1)):
            val -= (1 << num_bits)
        return val

    def _parse_rtcm3_payload(self, msg_id: int, payload: bytes) -> None:
        """Extracts satellite tracking masks and reference station coordinates directly from RTCM3."""
        try:
            # 1. Message 1005 / 1006: Base Station Antenna Reference Point (ARP)
            if msg_id in (1005, 1006) and len(payload) >= 19:
                self.rtcm_1005_count += 1
                raw_x = self._get_signed_bits(payload, 34, 38)
                raw_y = self._get_signed_bits(payload, 74, 38)
                raw_z = self._get_signed_bits(payload, 114, 38)
                x = raw_x * 0.0001
                y = raw_y * 0.0001
                z = raw_z * 0.0001
                if abs(x) > 1000.0 and abs(y) > 1000.0:
                    self._check_1005_consistency(x, y, z)
                    lat, lon, alt = self._ecef_to_lla(x, y, z)
                    if not self.is_static_fixed:
                        self.survey_lat = lat
                        self.survey_lon = lon
                        self.survey_alt = alt
                        # Only use 1005 as a fallback sample source when GGA isn't flowing,
                        # so the survey average isn't a mix of two different position sources.
                        if time.time() - self._last_gga_time > 5.0:
                            self._update_survey_statistics(lat, lon, alt)

            # 2. MSM Messages (1071-1077 GPS, 1081-1087 GLO, 1091-1097 GAL, 1111-1117 QZS, 1121-1127 BDS)
            elif 1071 <= msg_id <= 1137 and len(payload) >= 18:
                sat_mask = self._get_bits(payload, 73, 64)
                num_sats = bin(sat_mask).count('1')
                constellation = msg_id // 10
                self.msm_sats[constellation] = num_sats
                total_sats = sum(self.msm_sats.values())
                if total_sats > 0:
                    self.satellites_tracked = total_sats
        except Exception:
            pass

    def _check_1005_consistency(self, x: float, y: float, z: float) -> None:
        """
        Verifies the reference position the GNSS hardware actually broadcasts (RTCM 1005).
        If it differs from the locked coordinates or jumps between messages, rovers lose RTK FIX.
        """
        now = time.time()
        warn_ok = now - self._last_1005_warn_time > 30.0

        if self._last_1005_ecef is not None and self.is_static_fixed:
            jump = math.dist((x, y, z), self._last_1005_ecef)
            if jump > 0.01 and warn_ok:
                msg = f"⚠️ RTCM 1005 base position JUMPED by {jump * 100:.1f} cm - rovers will drop RTK FIX!"
                self._add_log(msg, "WARN")
                print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
                self._last_1005_warn_time = now
                warn_ok = False
        self._last_1005_ecef = (x, y, z)

        if self.is_static_fixed and self._locked_ecef is not None:
            offset = math.dist((x, y, z), self._locked_ecef)
            self.base_1005_offset_m = offset
            if offset > 0.05 and warn_ok:
                msg = (
                    f"⚠️ Hardware broadcasts a base position {offset:.2f} m away from the locked coordinates. "
                    f"The fixed-position command was not applied (module may need a restart/power-cycle)."
                )
                self._add_log(msg, "WARN")
                print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
                self._last_1005_warn_time = now

    def _parse_survey_line(self, line: str) -> None:
        """Parses Quectel LC29H Survey-In sentences and standard NMEA sentences."""
        try:
            if line.startswith(('$GNGGA', '$GPGGA', '$GAGGA', '$GBGGA', '$GLGGA')):
                parts = line.split(',')
                if len(parts) >= 10:
                    lat = self._parse_nmea_coordinate(parts[2], parts[3], False)
                    lon = self._parse_nmea_coordinate(parts[4], parts[5], True)
                    if not self.is_static_fixed and lat and lon:
                        self.survey_lat = lat
                        self.survey_lon = lon
                    if parts[7].isdigit():
                        self.satellites_tracked = int(parts[7])
                    if parts[8].replace('.', '', 1).isdigit():
                        self.hdop = float(parts[8])
                    if not self.is_static_fixed and parts[9]:
                        try:
                            msl_alt = float(parts[9])
                            geoid_sep = float(parts[11]) if len(parts) > 11 and parts[11] else 0.0
                            # GGA altitude is above mean sea level; ECEF / RTCM 1005 needs WGS-84
                            # ellipsoidal height = MSL height + geoid separation (~+44 m in Hungary)
                            self.survey_alt = msl_alt + geoid_sep
                        except ValueError:
                            pass

                    if lat and lon and self.satellites_tracked >= 4:
                        self._last_gga_time = time.time()
                        self._update_survey_statistics(lat, lon, self.survey_alt)

            elif 'GSV' in line:
                self._parse_gsv(line)

        except Exception:
            pass

    def _parse_gsv(self, line: str) -> None:
        """Parses multi-constellation NMEA GSV sentences for per-satellite SNR (C/N0), elevation, azimuth."""
        try:
            body = line.strip().lstrip('$').split('*')[0]
            parts = body.split(',')
            if len(parts) < 4:
                return

            talker = parts[0][:2].upper()
            prefix_map = {'GP': 'G', 'GL': 'R', 'GA': 'E', 'GB': 'B', 'BD': 'B', 'GQ': 'Q', 'GN': 'S'}
            const_map = {'GP': 'GPS', 'GL': 'GLONASS', 'GA': 'Galileo', 'GB': 'BeiDou', 'BD': 'BeiDou', 'GQ': 'QZSS', 'GN': 'GNSS'}
            prefix = prefix_map.get(talker, 'S')
            constellation = const_map.get(talker, 'GNSS')

            if parts[3].isdigit():
                in_view = int(parts[3])
                if in_view > 0:
                    self.satellites_tracked = max(self.satellites_tracked, in_view)

            n_fields = len(parts) - 4
            now = time.time()
            self._last_gsv_time = now

            for i in range(4, 4 + (n_fields // 4) * 4, 4):
                prn_str = parts[i].strip()
                if not prn_str or not prn_str.isdigit():
                    continue
                prn = int(prn_str)
                sat_id = f"{prefix}{prn:02d}"

                elev = None
                if i + 1 < len(parts) and parts[i + 1]:
                    try:
                        elev = float(parts[i + 1])
                    except ValueError:
                        pass

                az = None
                if i + 2 < len(parts) and parts[i + 2]:
                    try:
                        az = float(parts[i + 2])
                    except ValueError:
                        pass

                snr = None
                if i + 3 < len(parts) and parts[i + 3]:
                    try:
                        snr = float(parts[i + 3])
                    except ValueError:
                        pass

                with self.sat_snr_lock:
                    if snr is not None and snr > 0:
                        self.sat_snr_map[sat_id] = {
                            "id": sat_id,
                            "constellation": constellation,
                            "prn": prn,
                            "elev": int(elev) if elev is not None else None,
                            "az": int(az) if az is not None else None,
                            "snr": round(snr, 1),
                            "last_seen": now
                        }
                    elif sat_id in self.sat_snr_map:
                        self.sat_snr_map[sat_id]["last_seen"] = now
        except Exception:
            pass

    def _serial_reader_loop(self) -> None:
        """Reads RTCM3 binary data + NMEA sentences and non-blocking multicasts to rovers."""
        import serial

        while self.is_running:
            ser = None
            try:
                self._add_log(f"Opening Serial Port: {self.serial_port} @ {self.baud_rate} baud")
                ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                ser.reset_input_buffer()
                with self.ser_lock:
                    self.ser = ser
                self._init_base_gnss_hardware()
                self._add_log("Base GNSS UART active! Monitoring Survey-In & streaming RTCM3...")

                raw_byte_stream = bytearray()

                while self.is_running:
                    try:
                        count = ser.in_waiting
                        if count > 0:
                            chunk = ser.read(min(count, 4096))
                        else:
                            chunk = ser.read(1)
                    except Exception as read_err:
                        self._add_log(f"Serial read warning: {read_err}", "WARN")
                        time.sleep(0.05)
                        continue

                    if not chunk:
                        time.sleep(0.01)
                        continue

                    self.last_serial_time = time.time()
                    raw_byte_stream.extend(chunk)
                    if len(raw_byte_stream) > 32768:
                        raw_byte_stream = raw_byte_stream[-16384:]

                    rtcm_frames_to_send = []

                    # Demultiplex raw byte stream: extract clean RTCM3 packets & NMEA sentences
                    while len(raw_byte_stream) > 0:
                        # 1. RTCM3 binary frame starting with preamble 0xD3
                        if raw_byte_stream[0] == 0xD3:
                            if len(raw_byte_stream) < 3:
                                break  # Incomplete header; wait for next serial read
                            # The 6 reserved bits in byte 1 must be 0 for valid RTCM3
                            if (raw_byte_stream[1] & 0xFC) != 0:
                                del raw_byte_stream[0]
                                continue
                            msg_len = ((raw_byte_stream[1] & 0x03) << 8) | raw_byte_stream[2]
                            frame_len = msg_len + 6
                            if len(raw_byte_stream) < frame_len:
                                break  # Full frame hasn't arrived yet; wait for remaining bytes

                            # Validate CRC-24Q: on mismatch this 0xD3 was a false sync (or corrupted
                            # frame) -> skip one byte and resync instead of trusting a bogus length
                            expected_crc = (
                                (raw_byte_stream[frame_len - 3] << 16)
                                | (raw_byte_stream[frame_len - 2] << 8)
                                | raw_byte_stream[frame_len - 1]
                            )
                            if crc24q(raw_byte_stream[:frame_len - 3]) != expected_crc:
                                self.rtcm_crc_errors += 1
                                del raw_byte_stream[0]
                                continue

                            frame = bytes(raw_byte_stream[:frame_len])
                            del raw_byte_stream[:frame_len]

                            self.total_rtcm_bytes_read += len(frame)
                            self.rtcm_packet_count += 1

                            frame_payload = frame[3 : 3 + msg_len]
                            if len(frame_payload) >= 2:
                                msg_id = (frame_payload[0] << 4) | (frame_payload[1] >> 4)
                                self._parse_rtcm3_payload(msg_id, frame_payload)

                            rtcm_frames_to_send.append(frame)
                            continue

                        # 2. NMEA ASCII sentence starting with '$'
                        elif raw_byte_stream[0] == 0x24:  # ord('$')
                            nl_idx = raw_byte_stream.find(b'\n')
                            if nl_idx == -1:
                                if len(raw_byte_stream) > 256:
                                    del raw_byte_stream[0]
                                break
                            nmea_bytes = bytes(raw_byte_stream[:nl_idx + 1])
                            del raw_byte_stream[:nl_idx + 1]
                            clean_str = nmea_bytes.decode('ascii', errors='ignore').strip()
                            if clean_str:
                                self._parse_survey_line(clean_str)
                            continue

                        # 3. Discard extraneous bytes (CR, LF, noise) outside frames
                        else:
                            del raw_byte_stream[0]

                    # Multicast ONLY valid, clean RTCM3 packets to all active rover clients (never raw NMEA).
                    # Non-blocking: frames go into each rover's queue; its own thread does the socket send.
                    if rtcm_frames_to_send:
                        outgoing_data = b"".join(rtcm_frames_to_send)
                        with self.clients_lock:
                            client_metas = list(self.clients_map.values())

                        for meta in client_metas:
                            q = meta["queue"]
                            try:
                                q.put_nowait(outgoing_data)
                            except queue.Full:
                                # Rover link is stalled: drop the oldest (stale) corrections first
                                try:
                                    q.get_nowait()
                                except queue.Empty:
                                    pass
                                meta["dropped"] += 1
                                try:
                                    q.put_nowait(outgoing_data)
                                except queue.Full:
                                    pass

            except Exception as e:
                self._add_log(f"Serial Error: {e}. Retrying in 2 seconds...", "ERROR")
                time.sleep(2.0)
            finally:
                with self.ser_lock:
                    self.ser = None
                if ser and ser.is_open:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _diagnostic_logger_loop(self) -> None:
        """Prints periodic terminal status summaries every 10 seconds."""
        while self.is_running:
            time.sleep(10.0)
            with self.clients_lock:
                rovers = len(self.clients_map)

            rtcm_kb = self.total_rtcm_bytes_read / 1024.0

            if self.is_static_fixed:
                offset_str = f"{self.base_1005_offset_m * 100:.1f} cm" if self.base_1005_offset_m is not None else "n/a"
                msg = f"🎯 [STATIC FIXED BASE] Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}, {self.survey_alt:.1f}m) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count}, 1005 vs locked: {offset_str}, CRC errors: {self.rtcm_crc_errors})"
            elif self.survey_valid:
                msg = f"🎯 [BASE READY] Status: LOCKED (Accuracy: < {self.survey_accuracy:.2f}m) | Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count})"
            else:
                rem = max(0, self.survey_target_duration - self.survey_duration)
                mins = rem // 60
                secs = rem % 60
                msg = f"⏳ [CALIBRATING] {self.survey_duration}s/{self.survey_target_duration}s ({mins}m {secs}s left) | Est. Acc: {self.survey_accuracy:.2f}m | Sats: {self.satellites_tracked} | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count})"

            print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

            # Ensure GSV output is flowing if no GSV received in last 15s
            if self.ser and (time.time() - self._last_gsv_time > 15.0):
                self._send_gnss_cmd("PAIR062,3,1")

    def _get_battery_data(self) -> Optional[dict]:
        """Reads INA219 battery bus voltage, shunt current, and power with hardware 128x averaging and sliding window filter."""
        try:
            try:
                import smbus
            except ImportError:
                import smbus2 as smbus  # type: ignore
            bus = smbus.SMBus(1)
            try:
                # 1. Enable Hardware 128-sample ADC averaging in INA219 config register (0x00)
                if not self._ina219_configured:
                    try:
                        # 0x3FFF: 32V FSR, 320mV shunt, 128x averaging (68.1ms), continuous mode
                        bus.write_word_data(0x40, 0x00, ((0x3F) | (0xFF << 8)))
                        self._ina219_configured = True
                    except Exception:
                        pass

                # 2. Register 0x02: Bus Voltage (LSB = 4 mV)
                raw_v = bus.read_word_data(0x40, 0x02)
                raw_v_swapped = ((raw_v & 0xFF) << 8) | (raw_v >> 8)
                raw_v_val = ((raw_v_swapped >> 3) & 0x1FFF) * 0.004

                # 3. Register 0x01: Shunt Current (signed 16-bit, 10 µV LSB / 0.1 Ω = 0.1 mA)
                raw_s = bus.read_word_data(0x40, 0x01)
                raw_s_swapped = ((raw_s & 0xFF) << 8) | (raw_s >> 8)
                if raw_s_swapped >= 32768:
                    raw_s_swapped -= 65536
                raw_ma_val = raw_s_swapped * 0.1

                # 4. Step-change detection: Flush window immediately if charger is plugged/unplugged
                if self._batt_ma_history:
                    last_avg_ma = sum(self._batt_ma_history) / len(self._batt_ma_history)
                    if (last_avg_ma < -100 and raw_ma_val > 50) or (last_avg_ma > 50 and raw_ma_val < -100):
                        self._batt_v_history.clear()
                        self._batt_ma_history.clear()

                self._batt_v_history.append(raw_v_val)
                self._batt_ma_history.append(raw_ma_val)

                # 5. Sliding window moving average (~3 seconds smooth)
                avg_v = round(sum(self._batt_v_history) / len(self._batt_v_history), 2)
                avg_ma = round(sum(self._batt_ma_history) / len(self._batt_ma_history), 1)
                power_w = round(avg_v * (abs(avg_ma) / 1000.0), 2)

                return {
                    "voltage": avg_v,
                    "current_ma": avg_ma,
                    "power_w": power_w,
                    "raw_voltage": round(raw_v_val, 2),
                    "raw_current_ma": round(raw_ma_val, 1)
                }
            finally:
                bus.close()
        except Exception:
            return None

    @staticmethod
    def _get_cpu_temp() -> Optional[float]:
        """Reads Raspberry Pi CPU temperature in Celsius."""
        try:
            thermal_path = "/sys/class/thermal/thermal_zone0/temp"
            if os.path.exists(thermal_path):
                with open(thermal_path, "r") as f:
                    temp_raw = f.read().strip()
                return round(float(temp_raw) / 1000.0, 1)
        except Exception:
            pass
        return None

    def _init_button(self) -> None:
        """Initializes GPIO 17 button input with internal pull-up resistor (Active-LOW)."""
        # 1. Try gpiozero (official Raspberry Pi recommended)
        try:
            from gpiozero import Button
            self.button_dev = Button(self.button_pin, pull_up=True, bounce_time=0.05)
            self.button_backend = "gpiozero"
            return
        except Exception:
            pass

        # 2. Try RPi.GPIO
        try:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.button_backend = "rpi.gpio"
            return
        except Exception:
            pass

        # 3. Try gpiod
        try:
            import gpiod
            chip = gpiod.Chip('gpiochip0')
            line = chip.get_line(self.button_pin)
            line.request(consumer="conerobot-btn", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
            self.button_dev = line
            self.button_backend = "gpiod"
            return
        except Exception:
            pass

        self.button_backend = None

    def is_button_pressed(self) -> bool:
        """Returns True if button on GPIO 17 is actively pressed (pulled to GND)."""
        if self.button_backend == "gpiozero" and self.button_dev:
            try:
                return bool(self.button_dev.is_pressed)
            except Exception:
                return False
        elif self.button_backend == "rpi.gpio":
            try:
                import RPi.GPIO as GPIO
                return GPIO.input(self.button_pin) == GPIO.LOW
            except Exception:
                return False
        elif self.button_backend == "gpiod" and self.button_dev:
            try:
                return self.button_dev.get_value() == 0
            except Exception:
                return False
        return False

    def _button_monitor_loop(self) -> None:
        """Monitors GPIO 17 button: tracks hold duration and initiates safe OS shutdown if held >= 3s."""
        while self.is_running:
            try:
                pressed = self.is_button_pressed()
                now = time.time()
                if pressed:
                    if self._button_press_start is None:
                        self._button_press_start = now
                    held = now - self._button_press_start
                    self._button_held_sec = held
                    if held >= 3.0 and not self._shutdown_initiated:
                        self._shutdown_initiated = True
                        self._add_log("🛑 SHUTDOWN BUTTON HELD (3s): Safely halting Linux OS & unmounting SD card...", "WARN")
                        print("\n" + "!" * 75)
                        print("  🛑 SAFE SHUTDOWN INITIATED VIA HARDWARE BUTTON (3s HOLD)")
                        print("  • Linux is unmounting SD card and halting CPU.")
                        print("  • Wait for green ACT LED to blink 10 times and turn off.")
                        print("  • Then flip the toggle switch to OFF to cut booster battery drain.")
                        print("!" * 75 + "\n")
                        try:
                            import subprocess
                            subprocess.run(["sync"], timeout=2.0)
                        except Exception:
                            pass
                        try:
                            import subprocess
                            subprocess.Popen(["sudo", "shutdown", "-h", "now"])
                        except Exception as e:
                            self._add_log(f"Failed to execute shutdown command: {e}", "ERROR")
                else:
                    self._button_press_start = None
                    self._button_held_sec = 0.0
            except Exception:
                pass
            time.sleep(0.05)

    def get_status_json(self, client_host: Optional[str] = None) -> dict:
        """Generates real-time telemetry dictionary for HTTP dashboard."""
        with self.clients_lock:
            active_rovers = [
                {
                    "ip": meta["ip"],
                    "port": meta["port"],
                    "uptime_sec": int(time.time() - meta["connected_at"]),
                    "bytes_sent_kb": round(meta["bytes_sent"] / 1024.0, 1)
                }
                for meta in self.clients_map.values()
            ]

        with self.logs_lock:
            log_list = list(self.logs)[-30:]

        remaining_sec = max(0, self.survey_target_duration - self.survey_duration)
        remaining_str = f"{remaining_sec // 60}m {remaining_sec % 60:02d}s"
        loc_store = self._load_locations_store()

        time_since_serial = round(time.time() - self.last_serial_time, 1) if self.last_serial_time else None
        is_serial_live = time_since_serial is not None and time_since_serial < 2.5
        current_ip = self._get_local_ip()
        display_ip = client_host if (client_host and not client_host.startswith("127.") and not client_host.endswith(".local")) else (current_ip if (current_ip and not current_ip.startswith("127.")) else self.local_ip)
        batt = self._get_battery_data()

        now = time.time()
        with self.sat_snr_lock:
            stale_keys = [k for k, v in self.sat_snr_map.items() if now - v.get("last_seen", 0) > 5.0]
            for k in stale_keys:
                del self.sat_snr_map[k]
            sat_list = list(self.sat_snr_map.values())

        sat_list.sort(key=lambda s: (s.get("snr") or 0), reverse=True)
        avg_snr = round(sum(s["snr"] for s in sat_list) / len(sat_list), 1) if sat_list else 0.0
        max_snr = max((s["snr"] for s in sat_list), default=0.0)
        strong_count = sum(1 for s in sat_list if s["snr"] >= 38.0)
        good_count = sum(1 for s in sat_list if 28.0 <= s["snr"] < 38.0)
        weak_count = sum(1 for s in sat_list if s["snr"] < 28.0)

        const_counts = {}
        for s in sat_list:
            c = s.get("constellation", "Other")
            const_counts[c] = const_counts.get(c, 0) + 1

        return {
            "survey_status": "STATIC_FIXED" if self.is_static_fixed else self.survey_status,
            "survey_valid": self.survey_valid,
            "is_static_fixed": self.is_static_fixed,
            "locked_timestamp": self.locked_timestamp,
            "saved_coords": self._get_saved_coords_info(),
            "saved_locations": loc_store.get("locations", {}),
            "active_location": loc_store.get("active_location") or self.active_location,
            "cpu_temp": self._get_cpu_temp(),
            "battery_voltage": batt["voltage"] if batt else None,
            "battery_current_ma": batt["current_ma"] if batt else None,
            "battery_power_w": batt["power_w"] if batt else None,
            "button_pressed": self.is_button_pressed(),
            "button_held_sec": round(self._button_held_sec, 1),
            "button_shutting_down": self._shutdown_initiated,
            "button_backend": self.button_backend,
            "sat_snr_data": {
                "sats": sat_list,
                "count": len(sat_list),
                "avg_snr": avg_snr,
                "max_snr": max_snr,
                "strong_count": strong_count,
                "good_count": good_count,
                "weak_count": weak_count,
                "constellations": const_counts
            },
            "survey_duration": self.survey_duration,
            "survey_target_duration": self.survey_target_duration,
            "remaining_sec": remaining_sec,
            "remaining_str": remaining_str,
            "survey_accuracy": round(self.survey_accuracy, 2),
            "survey_target_accuracy": self.survey_target_accuracy,
            "latitude": round(self.survey_lat, 8),
            "longitude": round(self.survey_lon, 8),
            "altitude": round(self.survey_alt, 2),
            "satellites": self.satellites_tracked,
            "hdop": round(self.hdop, 2),
            "rtcm_ingested_kb": round(self.total_rtcm_bytes_read / 1024.0, 1),
            "rtcm_broadcasted_kb": round(self.total_bytes_sent / 1024.0, 1),
            "rtcm_1005_count": self.rtcm_1005_count,
            "rtcm_crc_errors": self.rtcm_crc_errors,
            "base_1005_offset_m": round(self.base_1005_offset_m, 4) if self.base_1005_offset_m is not None else None,
            "is_serial_alive": is_serial_live,
            "last_serial_ago_sec": time_since_serial,
            "active_rovers_count": len(active_rovers),
            "active_rovers": active_rovers,
            "mountpoint": self.mountpoint,
            "ntrip_port": self.server_port,
            "local_ip": display_ip,
            "logs": log_list
        }

    def _web_server_loop(self) -> None:
        """Hosts multi-threaded, instant-response HTTP Web Dashboard on port 8080."""
        caster_instance = self

        class DashboardHandler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def address_string(self) -> str:
                return str(self.client_address[0])

            def log_message(self, format, *args):
                pass

            def do_POST(self):
                if self.path == '/api/lock_now':
                    success = caster_instance.lock_now()
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200 if success else 400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/use_saved':
                    success = caster_instance.use_saved_now()
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200 if success else 400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/recalibrate':
                    success = caster_instance.recalibrate()
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/locations/save':
                    content_len = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_len).decode('utf-8') if content_len > 0 else "{}"
                    try:
                        name = json.loads(post_data).get("name", "").strip()
                    except Exception:
                        name = ""
                    success = caster_instance.save_named_location(name)
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200 if success else 400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/locations/load':
                    content_len = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_len).decode('utf-8') if content_len > 0 else "{}"
                    try:
                        name = json.loads(post_data).get("name", "").strip()
                    except Exception:
                        name = ""
                    success = caster_instance.load_named_location(name)
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200 if success else 400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/locations/delete':
                    content_len = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_len).decode('utf-8') if content_len > 0 else "{}"
                    try:
                        name = json.loads(post_data).get("name", "").strip()
                    except Exception:
                        name = ""
                    success = caster_instance.delete_named_location(name)
                    resp = json.dumps({"status": "ok" if success else "error"}).encode('utf-8')
                    self.send_response(200 if success else 400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                elif self.path == '/api/gsv/enable':
                    caster_instance._send_gnss_cmd("PAIR062,3,1")
                    caster_instance._add_log("📡 Manually requested NMEA GSV satellite signal stream ($PAIR062,3,1)")
                    resp = json.dumps({"status": "ok"}).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(resp)
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_GET(self):
                if self.path.startswith('/api/status'):
                    client_host = self.headers.get('Host', '').split(':')[0]
                    payload = json.dumps(caster_instance.get_status_json(client_host)).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(payload)
                else:
                    html_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "index.html")
                    try:
                        with open(html_file, 'rb') as f:
                            payload = f.read()
                    except Exception as e:
                        payload = f"<!DOCTYPE html><html><body><h3>Dashboard file not found: {e}</h3></body></html>".encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(payload)

        server = ThreadingHTTPServer(('0.0.0.0', self.web_port), DashboardHandler)
        server.daemon_threads = True
        try:
            server.serve_forever()
        except Exception:
            server.server_close()



def main() -> None:
    parser = argparse.ArgumentParser(description="Raspberry Pi RTK Base Station NTRIP Caster & Web Dashboard")
    parser.add_argument('--serial', type=str, default='/dev/ttyAMA0', help="Base GNSS UART port (default: /dev/ttyAMA0)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--port', type=int, default=2101, help="NTRIP server port (default: 2101)")
    parser.add_argument('--web-port', type=int, default=8080, help="Web Dashboard port (default: 8080)")
    parser.add_argument('--mountpoint', type=str, default='BASE', help="NTRIP mountpoint name (default: BASE)")
    parser.add_argument('--password', type=str, default='none', help="Optional authentication password")
    parser.add_argument('--survey-time', type=int, default=3600, help="Survey-In calibration target time in seconds (default: 3600 = 1 hr)")
    parser.add_argument('--survey-acc', type=float, default=0.5, help="Survey-In target accuracy in meters (default: 0.5m)")
    parser.add_argument('--recalibrate', action='store_true', help="Clear saved coordinates and force a fresh calibration")
    parser.add_argument('--use-saved', action='store_true', help="Explicitly lock using previously saved static coordinates on boot")
    parser.add_argument('--fixed-lat', type=float, default=None, help="Manual fixed latitude override")
    parser.add_argument('--fixed-lon', type=float, default=None, help="Manual fixed longitude override")
    parser.add_argument('--fixed-alt', type=float, default=None, help="Manual fixed altitude override")
    args = parser.parse_args()

    caster = NTRIPBaseCaster(
        serial_port=args.serial,
        baud_rate=args.baud,
        server_port=args.port,
        web_port=args.web_port,
        mountpoint=args.mountpoint,
        password=args.password,
        survey_duration=args.survey_time,
        survey_accuracy=args.survey_acc,
        recalibrate=args.recalibrate,
        use_saved=args.use_saved,
        fixed_lat=args.fixed_lat,
        fixed_lon=args.fixed_lon,
        fixed_alt=args.fixed_alt
    )

    try:
        caster.start()
    except KeyboardInterrupt:
        print("\nStopping NTRIP Base Caster...")
        caster.is_running = False
        sys.exit(0)


if __name__ == '__main__':
    main()
