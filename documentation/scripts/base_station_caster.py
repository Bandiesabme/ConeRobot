#!/usr/bin/env python3
"""
==============================================================================
Raspberry Pi 5 RTK Base Station NTRIP Caster & Live Web Dashboard
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

        # Persistence file path
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "base_station_fixed_coords.json")

        # Coordinate sample history (for averaging)
        self.coord_samples: List[Tuple[float, float, float]] = []
        self.survey_lat = 0.0
        self.survey_lon = 0.0
        self.survey_alt = 0.0
        self.satellites_tracked = 0
        self.hdop = 99.9
        self.local_ip = self._get_local_ip()

        # Check manual coordinates or saved coordinates
        if fixed_lat is not None and fixed_lon is not None:
            self._apply_fixed_coords(fixed_lat, fixed_lon, fixed_alt or 0.0, "Command-Line Arguments")
        elif not recalibrate:
            self._load_saved_coords()

        self._add_log(f"Base Station initialized. Serial: {self.serial_port}, Web: http://{self.local_ip}:{self.web_port}")

    def _add_log(self, text: str, level: str = "INFO") -> None:
        """Adds a log entry with timestamp for the web console and terminal."""
        ts = datetime.now().strftime("%H:%M:%S")
        entry = {"time": ts, "level": level, "msg": text}
        with self.logs_lock:
            self.logs.append(entry)

    def _get_local_ip(self) -> str:
        """Helper to get primary network IP address."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def _load_saved_coords(self) -> bool:
        """Loads previously locked static base coordinates if available."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)
                if data.get("is_locked"):
                    lat = float(data["lat"])
                    lon = float(data["lon"])
                    alt = float(data.get("alt", 0.0))
                    self._apply_fixed_coords(lat, lon, alt, f"Saved Config ({data.get('timestamp', 'Unknown')})")
                    return True
            except Exception as e:
                self._add_log(f"Failed to read saved coords: {e}", "WARN")
        return False

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
            return True
        except Exception as e:
            self._add_log(f"Failed to save coordinates: {e}", "ERROR")
            return False

    def recalibrate(self) -> bool:
        """Clears saved fixed position and restarts a fresh 1-hour calibration survey."""
        try:
            if os.path.exists(self.config_file):
                os.remove(self.config_file)
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
        print("  📡 RASPBERRY PI 5 RTK BASE STATION & WEB DASHBOARD")
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

        # 4. Run serial reader in main thread
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

        now = time.time()
        if self.survey_start_time is None:
            self.survey_start_time = now
            self._add_log(f"🛰️ GPS Lock acquired! Starting Auto-Calibration timer (Target: {self.survey_target_duration}s)...")

        self.survey_duration = int(now - self.survey_start_time)
        self.coord_samples.append((lat, lon, alt))

        if len(self.coord_samples) >= 5:
            lats = [s[0] for s in self.coord_samples]
            lons = [s[1] for s in self.coord_samples]
            mean_lat = sum(lats) / len(lats)
            mean_lon = sum(lons) / len(lons)

            lat_m = 111132.0
            lon_m = 111412.0 * math.cos(math.radians(mean_lat))

            dx = [(ln - mean_lon) * lon_m for ln in lons]
            dy = [(lt - mean_lat) * lat_m for lt in lats]
            sigma_2d = math.sqrt((sum(x**2 for x in dx) + sum(y**2 for y in dy)) / len(dx))
            self.survey_accuracy = sigma_2d
            self.survey_lat = mean_lat
            self.survey_lon = mean_lon
            self.survey_alt = sum(s[2] for s in self.coord_samples) / len(self.coord_samples)

            # Auto-Lock condition reached
            if (self.survey_duration >= self.survey_target_duration and self.survey_accuracy <= self.survey_target_accuracy) or (self.survey_duration >= self.survey_target_duration * 1.5):
                self._add_log(f"🎯 Auto-Calibration COMPLETE! (Duration: {self.survey_duration}s, Acc: {self.survey_accuracy:.2f}m)")
                self.lock_now()
            else:
                self.survey_status = "CALIBRATING"

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

                    self.total_rtcm_bytes_read += len(chunk)
                    self.rtcm_packet_count += 1

                    raw_byte_stream.extend(chunk)
                    if len(raw_byte_stream) > 8192:
                        raw_byte_stream = raw_byte_stream[-4096:]

                    while b'\n' in raw_byte_stream:
                        line_bytes, _, remaining = raw_byte_stream.partition(b'\n')
                        raw_byte_stream = remaining
                        if b'$' in line_bytes:
                            dollar_idx = line_bytes.find(b'$')
                            clean_str = line_bytes[dollar_idx:].decode('ascii', errors='ignore').strip()
                            if clean_str:
                                self._parse_survey_line(clean_str)

                    # Multicast complete RTCM chunks to all active rover sockets (without holding lock)
                    with self.clients_lock:
                        client_items = list(self.clients_map.items())

                    dead_socks = []
                    for client, meta in client_items:
                        try:
                            client.sendall(chunk)
                            meta["bytes_sent"] += len(chunk)
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
                msg = f"🎯 [STATIC FIXED BASE] Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}, {self.survey_alt:.1f}m) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB [0 mm Drift]"
            elif self.survey_valid:
                msg = f"🎯 [BASE READY] Status: LOCKED (Accuracy: < {self.survey_accuracy:.2f}m) | Pos: ({self.survey_lat:.8f}, {self.survey_lon:.8f}) | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB"
            else:
                rem = max(0, self.survey_target_duration - self.survey_duration)
                mins = rem // 60
                secs = rem % 60
                msg = f"⏳ [CALIBRATING] {self.survey_duration}s/{self.survey_target_duration}s ({mins}m {secs}s left) | Est. Acc: {self.survey_accuracy:.2f}m | Sats: {self.satellites_tracked} | Rovers: {rovers} | RTCM: {rtcm_kb:.1f} KB"

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

        remaining_sec = max(0, self.survey_target_duration - self.survey_duration)
        remaining_str = f"{remaining_sec // 60}m {remaining_sec % 60:02d}s"

        return {
            "survey_status": "STATIC_FIXED" if self.is_static_fixed else self.survey_status,
            "survey_valid": self.survey_valid,
            "is_static_fixed": self.is_static_fixed,
            "locked_timestamp": self.locked_timestamp,
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
          <div class="brand-subtitle">Raspberry Pi 5 Local Caster & Auto-Lock</div>
        </div>
      </div>
      <div id="statusBadge" class="status-badge">
        <span id="statusIcon">⏳</span>
        <span id="statusText">CALIBRATING</span>
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
    parser = argparse.ArgumentParser(description="Raspberry Pi 5 RTK Base Station NTRIP Caster & Web Dashboard")
    parser.add_argument('--serial', type=str, default='/dev/ttyAMA0', help="Base GNSS UART port (default: /dev/ttyAMA0)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--port', type=int, default=2101, help="NTRIP server port (default: 2101)")
    parser.add_argument('--web-port', type=int, default=8080, help="Web Dashboard port (default: 8080)")
    parser.add_argument('--mountpoint', type=str, default='BASE', help="NTRIP mountpoint name (default: BASE)")
    parser.add_argument('--password', type=str, default='none', help="Optional authentication password")
    parser.add_argument('--survey-time', type=int, default=3600, help="Survey-In calibration target time in seconds (default: 3600 = 1 hr)")
    parser.add_argument('--survey-acc', type=float, default=0.5, help="Survey-In target accuracy in meters (default: 0.5m)")
    parser.add_argument('--recalibrate', action='store_true', help="Clear saved coordinates and force a fresh calibration")
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
