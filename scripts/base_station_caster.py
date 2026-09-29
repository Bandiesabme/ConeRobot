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
import select
import socket
import sys
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple


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
            with open(self.locations_file, "w") as f:
                json.dump(store, f, indent=2)
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
            with open(self.config_file, "w") as f:
                json.dump({
                    "is_locked": True,
                    "lat": round(lat, 8),
                    "lon": round(lon, 8),
                    "alt": round(alt, 2),
                    "samples": len(self.coord_samples) or 100,
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }, f, indent=2)
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
            with open(self.config_file, "w") as f:
                json.dump({
                    "is_locked": True,
                    "lat": round(mean_lat, 8),
                    "lon": round(mean_lon, 8),
                    "alt": round(mean_alt, 2),
                    "samples": len(self.coord_samples),
                    "timestamp": self.locked_timestamp
                }, f, indent=2)
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

        # 5. Run serial reader in main thread
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
            client_sock.settimeout(2.0)
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # Deduplicate & register
            with self.clients_lock:
                to_remove = []
                for sock, meta in self.clients_map.items():
                    if meta["ip"] == ip:
                        to_remove.append(sock)
                for sock in to_remove:
                    try:
                        sock.close()
                    except Exception:
                        pass
                    del self.clients_map[sock]

                self.clients_map[client_sock] = {
                    "ip": ip,
                    "port": port,
                    "connected_at": time.time(),
                    "bytes_sent": 0
                }

            self._add_log(f"Stream Active: Broadcasting RTCM3 to Rover {ip}")

        except Exception as e:
            self._add_log(f"Client handshake error ({ip}): {e}", "WARN")
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
                    lat, lon, alt = self._ecef_to_lla(x, y, z)
                    if not self.is_static_fixed:
                        self.survey_lat = lat
                        self.survey_lon = lon
                        self.survey_alt = alt
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
                    if not self.is_static_fixed and parts[9].replace('.', '', 1).replace('-', '', 1).isdigit():
                        self.survey_alt = float(parts[9])

                    if lat and lon and self.satellites_tracked >= 4:
                        self._update_survey_statistics(lat, lon, self.survey_alt)

            elif 'GSV' in line:
                parts = line.split(',')
                if len(parts) >= 4 and parts[3].isdigit():
                    sats_in_view = int(parts[3])
                    if sats_in_view > 0:
                        self.satellites_tracked = max(self.satellites_tracked, sats_in_view)

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

                    # Multicast ONLY valid, clean RTCM3 packets to all active rover clients (never raw NMEA)
                    if rtcm_frames_to_send:
                        outgoing_data = b"".join(rtcm_frames_to_send)
                        with self.clients_lock:
                            client_items = list(self.clients_map.items())

                        dead_socks = []
                        for client, meta in client_items:
                            try:
                                client.sendall(outgoing_data)
                                meta["bytes_sent"] += len(outgoing_data)
                            except (socket.error, socket.timeout):
                                dead_socks.append(client)

                        if dead_socks:
                            with self.clients_lock:
                                for dead in dead_socks:
                                    if dead in self.clients_map:
                                        del self.clients_map[dead]
                                    try:
                                        dead.close()
                                    except Exception:
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
                msg = f"🎯 [STATIC FIXED BASE] Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}, {self.survey_alt:.1f}m) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count}) [0 mm Drift]"
            elif self.survey_valid:
                msg = f"🎯 [BASE READY] Status: LOCKED (Accuracy: < {self.survey_accuracy:.2f}m) | Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count})"
            else:
                rem = max(0, self.survey_target_duration - self.survey_duration)
                mins = rem // 60
                secs = rem % 60
                msg = f"⏳ [CALIBRATING] {self.survey_duration}s/{self.survey_target_duration}s ({mins}m {secs}s left) | Est. Acc: {self.survey_accuracy:.2f}m | Sats: {self.satellites_tracked} | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB (Msg 1005: {self.rtcm_1005_count})"

            print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

    def get_status_json(self) -> dict:
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

    def get_status_json(self) -> dict:
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

        return {
            "survey_status": "STATIC_FIXED" if self.is_static_fixed else self.survey_status,
            "survey_valid": self.survey_valid,
            "is_static_fixed": self.is_static_fixed,
            "locked_timestamp": self.locked_timestamp,
            "saved_coords": self._get_saved_coords_info(),
            "saved_locations": loc_store.get("locations", {}),
            "active_location": loc_store.get("active_location") or self.active_location,
            "cpu_temp": self._get_cpu_temp(),
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
            "active_rovers_count": len(active_rovers),
            "active_rovers": active_rovers,
            "mountpoint": self.mountpoint,
            "ntrip_port": self.server_port,
            "local_ip": self.local_ip,
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
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_GET(self):
                if self.path.startswith('/api/status'):
                    payload = json.dumps(caster_instance.get_status_json()).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(payload)
                else:
                    payload = DASHBOARD_HTML.encode('utf-8')
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


# 100% Offline-Ready HTML5 / CSS / Vanilla JS Web Dashboard
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ConeRobot RTK Base Station Dashboard</title>
  <style>
    :root {
      --bg: #0b0f19;
      --card-bg: rgba(23, 32, 54, 0.75);
      --card-border: rgba(56, 189, 248, 0.15);
      --accent: #38bdf8;
      --accent-glow: rgba(56, 189, 248, 0.35);
      --success: #10b981;
      --success-glow: rgba(16, 185, 129, 0.3);
      --warning: #f59e0b;
      --text: #f1f5f9;
      --text-muted: #94a3b8;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg);
      color: var(--text);
      padding: 24px;
      line-height: 1.5;
    }
    .container { max-width: 1200px; margin: 0 auto; }
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 24px;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--card-border);
    }
    .brand { display: flex; align-items: center; gap: 12px; }
    .brand-icon { font-size: 32px; }
    .brand-title { font-size: 24px; font-weight: 700; color: #fff; letter-spacing: -0.5px; }
    .brand-subtitle { font-size: 13px; color: var(--text-muted); }
    .status-badge {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 8px 16px;
      border-radius: 9999px;
      font-size: 14px;
      font-weight: 600;
      background: rgba(245, 158, 11, 0.15);
      color: var(--warning);
      border: 1px solid rgba(245, 158, 11, 0.3);
    }
    .status-badge.locked {
      background: rgba(16, 185, 129, 0.15);
      color: var(--success);
      border-color: rgba(16, 185, 129, 0.3);
      box-shadow: 0 0 15px var(--success-glow);
    }
    .status-badge.fixed {
      background: rgba(56, 189, 248, 0.15);
      color: var(--accent);
      border-color: rgba(56, 189, 248, 0.3);
      box-shadow: 0 0 15px var(--accent-glow);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
      gap: 20px;
      margin-bottom: 20px;
    }
    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 16px;
      padding: 20px;
      backdrop-filter: blur(12px);
    }
    .card-title {
      font-size: 15px;
      font-weight: 600;
      color: var(--text-muted);
      margin-bottom: 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .metric-value { font-size: 32px; font-weight: 700; color: #fff; margin-bottom: 4px; }
    .metric-unit { font-size: 16px; color: var(--text-muted); margin-left: 4px; }
    .data-row {
      display: flex;
      justify-content: space-between;
      padding: 8px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
      font-size: 14px;
    }
    .data-row:last-child { border-bottom: none; }
    .data-label { color: var(--text-muted); }
    .data-val { font-weight: 600; font-family: ui-monospace, monospace; }
    .progress-container { margin: 16px 0; }
    .progress-bar-bg {
      height: 8px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 4px;
      overflow: hidden;
    }
    .progress-bar-fill {
      height: 100%;
      background: linear-gradient(90deg, #38bdf8, #818cf8);
      width: 0%;
      transition: width 0.3s ease;
    }
    .progress-bar-fill.complete {
      background: linear-gradient(90deg, #10b981, #34d399);
    }
    .btn {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      padding: 8px 14px;
      background: #0284c7;
      color: #fff;
      text-decoration: none;
      border: none;
      cursor: pointer;
      border-radius: 8px;
      font-weight: 600;
      font-size: 13px;
      transition: background 0.2s;
    }
    .btn:hover { background: #0369a1; }
    .btn-warning { background: #d97706; }
    .btn-warning:hover { background: #b45309; }
    .btn-secondary { background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255, 255, 255, 0.15); }
    .btn-secondary:hover { background: rgba(255, 255, 255, 0.2); }
    .button-group { display: flex; gap: 10px; margin-top: 14px; }
    .code-box {
      background: rgba(0, 0, 0, 0.4);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 8px;
      padding: 10px;
      font-family: ui-monospace, monospace;
      font-size: 12px;
      color: #38bdf8;
      white-space: pre;
      overflow-x: auto;
      margin-top: 8px;
    }
    .terminal-box {
      background: #050811;
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 12px;
      font-family: ui-monospace, monospace;
      font-size: 12px;
      color: #a5f3fc;
      height: 200px;
      overflow-y: auto;
    }
    .log-line { margin-bottom: 4px; }
    .log-time { color: #64748b; margin-right: 8px; }
    .log-msg.error { color: #f87171; }
    .log-msg.warn { color: #fbbf24; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div class="brand">
        <div class="brand-icon">📡</div>
        <div>
          <div class="brand-title">RTK Base Station</div>
          <div class="brand-subtitle">Raspberry Pi Local Caster & Auto-Lock</div>
        </div>
      </div>
      <div style="display: flex; align-items: center; gap: 12px;">
        <div id="cpuBadge" class="status-badge" style="background: rgba(255, 255, 255, 0.05); color: var(--text-muted); border-color: rgba(255, 255, 255, 0.1);">
          <span>🌡️ CPU:</span>
          <span id="cpuTempVal" style="color: #34d399; font-family: ui-monospace, monospace; font-weight: 700;">-- °C</span>
        </div>
        <div id="statusBadge" class="status-badge">
          <span id="statusIcon">⏳</span>
          <span id="statusText">CALIBRATING</span>
        </div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="card-title">
          <span>🎯 SURVEY-IN / STATIC CALIBRATION</span>
          <span id="surveyPercent" class="data-val" style="color: var(--accent);">0%</span>
        </div>
        <div class="progress-container">
          <div class="progress-bar-bg">
            <div id="surveyProgressBar" class="progress-bar-fill"></div>
          </div>
        </div>
        <div class="data-row">
          <span class="data-label">⏳ Time Remaining:</span>
          <span id="surveyRemaining" class="data-val">Calibrating...</span>
        </div>
        <div class="data-row">
          <span class="data-label">Elapsed Duration:</span>
          <span id="surveyTime" class="data-val">0s / 3600s</span>
        </div>
        <div class="data-row">
          <span class="data-label">Live Accuracy StdDev (σ):</span>
          <span id="surveyAcc" class="data-val">-- m</span>
        </div>
        <div class="data-row">
          <span class="data-label">Anchor Reference Status:</span>
          <span id="anchorStatus" class="data-val" style="color: var(--warning);">Converging...</span>
        </div>
        <div class="button-group">
          <button id="lockNowBtn" onclick="lockPositionNow()" class="btn btn-warning">🔒 Lock Position Now</button>
          <button id="useSavedBtn" onclick="useSavedCoords()" class="btn btn-secondary" style="display: none;">📌 Use Saved Position</button>
          <button id="recalBtn" onclick="recalibrateBase()" class="btn btn-secondary">🔄 Recalibrate</button>
        </div>
      </div>

      <div class="card">
        <div class="card-title">🛰️ GNSS Satellite Lock</div>
        <div style="display: flex; gap: 24px; margin-bottom: 12px;">
          <div>
            <div class="data-label">Satellites Tracked</div>
            <div class="metric-value"><span id="satsCount">0</span><span class="metric-unit">sats</span></div>
          </div>
          <div>
            <div class="data-label">HDOP Quality</div>
            <div class="metric-value"><span id="hdopVal">--</span></div>
          </div>
        </div>
        <div class="data-row">
          <span class="data-label">Constellations:</span>
          <span class="data-val">GPS + GLO + GAL + BDS (L1/L5)</span>
        </div>
        <div class="data-row">
          <span class="data-label">Raw RTCM3 Ingested:</span>
          <span id="rtcmIngested" class="data-val">0.0 KB</span>
        </div>
        <div class="data-row">
          <span class="data-label">Broadcasted Throughput:</span>
          <span id="rtcmBroadcast" class="data-val">0.0 KB</span>
        </div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="card-title">📍 Base Station Coordinates</div>
        <div class="data-row">
          <span class="data-label">Latitude:</span>
          <span id="baseLat" class="data-val">0.00000000°</span>
        </div>
        <div class="data-row">
          <span class="data-label">Longitude:</span>
          <span id="baseLon" class="data-val">0.00000000°</span>
        </div>
        <div class="data-row">
          <span class="data-label">Elevation / Altitude:</span>
          <span id="baseAlt" class="data-val">0.00 m</span>
        </div>
        <div style="margin-top: 14px;">
          <a id="mapsBtn" href="#" target="_blank" class="btn">🗺️ Open in Google Maps</a>
        </div>
      </div>

      <div class="card">
        <div class="card-title">
          <span>📡 Connected Rovers</span>
          <span id="roversCount" class="data-val" style="color: var(--accent);">0 Active</span>
        </div>
        <div class="data-row">
          <span class="data-label">NTRIP Caster Port:</span>
          <span id="ntripPort" class="data-val">2101</span>
        </div>
        <div class="data-row">
          <span class="data-label">Mountpoint:</span>
          <span id="mountpointVal" class="data-val">/BASE</span>
        </div>
        <div class="data-label" style="margin-top: 10px;">Rover Configuration Snippet:</div>
        <div id="configSnippet" class="code-box">Loading...</div>
      </div>
    </div>

    <div class="grid">
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-title">
          <span>📍 Saved Benchmark Locations (Instant 0 mm Profiles)</span>
          <span id="activePresetBadge" class="data-val" style="color: var(--accent); font-size: 13px;">No Preset Active</span>
        </div>
        <div style="display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin-top: 8px;">
          <div style="flex: 1; min-width: 280px;">
            <label class="data-label" style="display: block; margin-bottom: 6px;">Select Saved Base Spot:</label>
            <select id="locationsSelect" style="width: 100%; padding: 9px 12px; background: rgba(0,0,0,0.6); border: 1px solid var(--card-border); border-radius: 8px; color: #fff; font-size: 13px; font-family: ui-monospace, monospace;">
              <option value="">-- No Saved Locations --</option>
            </select>
          </div>
          <div style="display: flex; gap: 8px; align-items: flex-end; margin-top: 18px;">
            <button onclick="loadSelectedLocation()" class="btn" style="background: #0284c7;">📌 Load Selected Spot</button>
            <button onclick="saveCurrentLocationPrompt()" class="btn btn-warning">💾 Save Current Spot As...</button>
            <button onclick="deleteSelectedLocation()" class="btn btn-secondary" style="color: #f87171;">🗑️ Delete</button>
          </div>
        </div>
      </div>
    </div>

    <div class="grid">
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-title">
          <span>🖥️ Live Base Station Console & NMEA Logs</span>
          <span class="data-val" style="font-size: 11px; opacity: 0.7;">Auto-refreshing</span>
        </div>
        <div id="terminalBox" class="terminal-box">
          <div class="log-line"><span class="log-msg">Connecting to live log stream...</span></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    let userScrolled = false;
    const term = document.getElementById('terminalBox');
    term.addEventListener('scroll', () => {
      userScrolled = (term.scrollHeight - term.scrollTop - term.clientHeight) > 20;
    });

    async function loadSelectedLocation() {
      const select = document.getElementById('locationsSelect');
      const name = select.value;
      if (!name) {
        alert('Please select a saved location first.');
        return;
      }
      if (confirm(`Instantly switch to saved benchmark '${name}' (0 mm drift)?`)) {
        try {
          const res = await fetch('/api/locations/load', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name })
          });
          const json = await res.json();
          if (json.status === 'ok') {
            updateDashboard();
          } else {
            alert('Failed to load location.');
          }
        } catch (e) {
          alert('Error: ' + e);
        }
      }
    }

    async function saveCurrentLocationPrompt() {
      const name = prompt('Enter a name for this benchmark position (e.g. Home Yard, University Rooftop, Test Track Spot A):');
      if (!name || !name.trim()) return;
      try {
        const res = await fetch('/api/locations/save', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name: name.trim() })
        });
        const json = await res.json();
        if (json.status === 'ok') {
          updateDashboard();
        } else {
          alert('Failed to save location.');
        }
      } catch (e) {
        alert('Error: ' + e);
      }
    }

    async function deleteSelectedLocation() {
      const select = document.getElementById('locationsSelect');
      const name = select.value;
      if (!name) {
        alert('Please select a saved location to delete.');
        return;
      }
      if (confirm(`Are you sure you want to delete preset '${name}'?`)) {
        try {
          const res = await fetch('/api/locations/delete', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name })
          });
          const json = await res.json();
          if (json.status === 'ok') {
            updateDashboard();
          } else {
            alert('Failed to delete location.');
          }
        } catch (e) {
          alert('Error: ' + e);
        }
      }
    }

    async function lockPositionNow() {
      if (confirm('Lock the current averaged position as the permanent static base coordinate (0 mm drift)?')) {
        try {
          await fetch('/api/lock_now', { method: 'POST' });
          updateDashboard();
        } catch (e) {
          alert('Lock failed: ' + e);
        }
      }
    }

    async function useSavedCoords() {
      if (confirm('Apply saved static base coordinates from previous session (0 mm drift)?')) {
        try {
          const res = await fetch('/api/use_saved', { method: 'POST' });
          const json = await res.json();
          if (json.status === 'ok') {
            updateDashboard();
          } else {
            alert('Failed to apply saved coordinates.');
          }
        } catch (e) {
          alert('Error: ' + e);
        }
      }
    }

    async function recalibrateBase() {
      if (confirm('Clear saved coordinates and start a fresh 1-hour calibration survey?')) {
        try {
          await fetch('/api/recalibrate', { method: 'POST' });
          updateDashboard();
        } catch (e) {
          alert('Recalibrate failed: ' + e);
        }
      }
    }

    async function updateDashboard() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();

        const badge = document.getElementById('statusBadge');
        const statusText = document.getElementById('statusText');
        const statusIcon = document.getElementById('statusIcon');
        const progressBar = document.getElementById('surveyProgressBar');
        const anchorStatus = document.getElementById('anchorStatus');
        const remainingEl = document.getElementById('surveyRemaining');
        const lockBtn = document.getElementById('lockNowBtn');
        const useSavedBtn = document.getElementById('useSavedBtn');

        if (data.is_static_fixed) {
          badge.className = 'status-badge fixed';
          statusIcon.textContent = '🎯';
          statusText.textContent = 'STATIC FIXED BASE (0 mm Drift)';
          progressBar.className = 'progress-bar-fill complete';
          progressBar.style.width = '100%';
          document.getElementById('surveyPercent').textContent = '100%';
          anchorStatus.textContent = 'PERMANENT STATIC LOCKED (0 mm)';
          anchorStatus.style.color = 'var(--accent)';
          remainingEl.textContent = `✅ Saved ${data.locked_timestamp || 'Active'}`;
          remainingEl.style.color = 'var(--accent)';
          lockBtn.style.display = 'none';
          useSavedBtn.style.display = 'none';
        } else if (data.survey_valid) {
          badge.className = 'status-badge locked';
          statusIcon.textContent = '🎯';
          statusText.textContent = 'CALIBRATION COMPLETE';
          progressBar.className = 'progress-bar-fill complete';
          progressBar.style.width = '100%';
          document.getElementById('surveyPercent').textContent = '100%';
          anchorStatus.textContent = 'LOCKED & VALID';
          anchorStatus.style.color = 'var(--success)';
          remainingEl.textContent = '✅ Auto-Locking...';
          remainingEl.style.color = 'var(--success)';
          lockBtn.style.display = 'inline-flex';
          useSavedBtn.style.display = 'none';
        } else {
          badge.className = 'status-badge';
          statusIcon.textContent = '⏳';
          statusText.textContent = `CALIBRATING (${data.remaining_str} left)`;
          progressBar.className = 'progress-bar-fill';
          const pct = Math.min(100, Math.round((data.survey_duration / data.survey_target_duration) * 100));
          progressBar.style.width = pct + '%';
          document.getElementById('surveyPercent').textContent = pct + '%';
          anchorStatus.textContent = `Converging Samples (${data.survey_duration}s)...`;
          anchorStatus.style.color = 'var(--warning)';
          remainingEl.textContent = `${data.remaining_str} remaining`;
          remainingEl.style.color = '#38bdf8';
          lockBtn.style.display = 'inline-flex';

          if (data.saved_coords) {
            useSavedBtn.style.display = 'inline-flex';
            useSavedBtn.textContent = `📌 Use Saved (${data.saved_coords.lat.toFixed(5)}°, ${data.saved_coords.lon.toFixed(5)}°)`;
          } else {
            useSavedBtn.style.display = 'none';
          }
        }

        if (data.cpu_temp !== null && data.cpu_temp !== undefined) {
          const tempEl = document.getElementById('cpuTempVal');
          tempEl.textContent = `${data.cpu_temp.toFixed(1)} °C`;
          if (data.cpu_temp >= 75) {
            tempEl.style.color = '#f87171';
          } else if (data.cpu_temp >= 60) {
            tempEl.style.color = '#fbbf24';
          } else {
            tempEl.style.color = '#34d399';
          }
        }

        document.getElementById('surveyTime').textContent = `${data.survey_duration}s / ${data.survey_target_duration}s`;
        document.getElementById('surveyAcc').textContent = `${data.survey_accuracy.toFixed(2)} m`;
        document.getElementById('satsCount').textContent = data.satellites;
        document.getElementById('hdopVal').textContent = data.hdop.toFixed(2);
        document.getElementById('rtcmIngested').textContent = `${data.rtcm_ingested_kb.toFixed(1)} KB`;
        document.getElementById('rtcmBroadcast').textContent = `${data.rtcm_broadcasted_kb.toFixed(1)} KB`;

        document.getElementById('baseLat').textContent = data.latitude.toFixed(8) + '°';
        document.getElementById('baseLon').textContent = data.longitude.toFixed(8) + '°';
        document.getElementById('baseAlt').textContent = data.altitude.toFixed(2) + ' m';
        document.getElementById('mapsBtn').href = `https://www.google.com/maps?q=${data.latitude},${data.longitude}`;

        document.getElementById('roversCount').textContent = `${data.active_rovers_count} Active`;
        document.getElementById('ntripPort').textContent = data.ntrip_port;
        document.getElementById('mountpointVal').textContent = `/${data.mountpoint}`;

        document.getElementById('configSnippet').textContent = 
`ntrip_caster: "${data.local_ip}"
ntrip_port: ${data.ntrip_port}
ntrip_mountpoint: "${data.mountpoint}"`;

        // Populate Saved Benchmark Locations dropdown
        const locSelect = document.getElementById('locationsSelect');
        const activeBadge = document.getElementById('activePresetBadge');
        if (locSelect && activeBadge) {
          const savedLocs = data.saved_locations || {};
          const locKeys = Object.keys(savedLocs);

          if (data.active_location) {
            activeBadge.textContent = `🎯 Active Benchmark: ${data.active_location}`;
            activeBadge.style.color = 'var(--success)';
          } else {
            activeBadge.textContent = 'No Preset Active (Calibrating)';
            activeBadge.style.color = 'var(--text-muted)';
          }

          const prevKeys = Array.from(locSelect.options).map(o => o.value).filter(v => v !== '');
          const keysMatch = prevKeys.length === locKeys.length && prevKeys.every((v, i) => v === locKeys[i]);

          if (!keysMatch || locSelect.options.length === 0) {
            const currentVal = locSelect.value;
            locSelect.innerHTML = '';
            if (locKeys.length === 0) {
              locSelect.innerHTML = '<option value="">-- No Saved Locations Yet --</option>';
            } else {
              locKeys.forEach(k => {
                const loc = savedLocs[k];
                const opt = document.createElement('option');
                opt.value = k;
                opt.textContent = `${k} [${loc.lat.toFixed(6)}°, ${loc.lon.toFixed(6)}° | ${loc.alt}m]`;
                if (k === (data.active_location || currentVal)) {
                  opt.selected = true;
                }
                locSelect.appendChild(opt);
              });
            }
          }
        }

        if (data.logs && data.logs.length > 0) {
          term.innerHTML = data.logs.map(l => {
            const cls = l.level === 'ERROR' ? 'error' : (l.level === 'WARN' ? 'warn' : '');
            return `<div class="log-line"><span class="log-time">[${l.time}]</span><span class="log-msg ${cls}">${l.msg}</span></div>`;
          }).join('');
          if (!userScrolled) {
            term.scrollTop = term.scrollHeight;
          }
        }

      } catch (err) {
        console.error('Failed to fetch status:', err);
      }
    }

    setInterval(updateDashboard, 1000);
    updateDashboard();
  </script>
</body>
</html>
"""


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
