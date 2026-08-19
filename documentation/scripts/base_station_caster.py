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
      3. Instant client deduplication & stale connection pruning.
      4. Multi-threaded, lightning-fast HTTP Web Dashboard on port 8080.
      5. 100% offline-ready (zero external CDN or font dependencies).

Usage:
    python3 base_station_caster.py --port 2101 --web-port 8080 --mountpoint BASE --serial /dev/ttyAMA0
==============================================================================
"""

import argparse
from collections import deque
from datetime import datetime
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
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
        password: str = "none"
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

        # Survey-In & Statistical Estimator
        self.survey_target_duration = 300  # 5 minutes
        self.survey_target_accuracy = 2.0   # < 2 meters
        self.survey_start_time: Optional[float] = None
        self.survey_duration = 0
        self.survey_status = "INITIALIZING"
        self.survey_valid = False
        self.survey_accuracy = 99.9

        # Coordinate sample history
        self.coord_samples: Deque[Tuple[float, float, float]] = deque(maxlen=600)
        self.survey_lat = 0.0
        self.survey_lon = 0.0
        self.survey_alt = 0.0
        self.satellites_tracked = 0
        self.hdop = 99.9
        self.local_ip = self._get_local_ip()

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

    def start(self) -> None:
        """Starts NTRIP Caster, Web Dashboard, and Serial Reader threads."""
        print("=" * 75)
        print("  📡 RASPBERRY PI 5 RTK BASE STATION & WEB DASHBOARD")
        print("=" * 75)
        print(f"  • Base Station IP   : {self.local_ip}")
        print(f"  • Serial Port       : {self.serial_port} @ {self.baud_rate} baud")
        print(f"  • NTRIP Server Port : {self.server_port} (Mountpoint: /{self.mountpoint})")
        print(f"  • 🌐 Web Dashboard  : http://{self.local_ip}:{self.web_port}")
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
        finally:
            server_sock.close()

    def _handle_client_handshake(self, client_sock: socket.socket, client_addr: tuple) -> None:
        """Handles standard NTRIP 1.0/2.0 HTTP header handshake and deduplicates connections."""
        client_sock.settimeout(5.0)
        rover_ip = client_addr[0]
        rover_port = client_addr[1]
        addr_str = f"{rover_ip}:{rover_port}"

        try:
            req_data = b""
            while b"\r\n\r\n" not in req_data and b"\n\n" not in req_data:
                chunk = client_sock.recv(1024)
                if not chunk:
                    client_sock.close()
                    return
                req_data += chunk
                if len(req_data) > 4096:
                    break

            req_text = req_data.decode('latin1', errors='ignore')
            first_line = req_text.splitlines()[0] if req_text else ""
            self._add_log(f"Rover handshake from {addr_str}: {first_line}")

            if f"/{self.mountpoint}" not in first_line and f"/{self.mountpoint.lower()}" not in first_line:
                self._add_log(f"Rover requested unknown mountpoint: {first_line}", "WARN")
                client_sock.sendall(b"HTTP/1.0 404 Not Found\r\n\r\n")
                client_sock.close()
                return

            client_sock.sendall(b"ICY 200 OK\r\n\r\n")
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client_sock.settimeout(2.0)

            with self.clients_lock:
                # Deduplicate: Clean up any older ghost sockets from the same IP
                stale_socks = [s for s, m in self.clients_map.items() if m["ip"] == rover_ip]
                for stale in stale_socks:
                    del self.clients_map[stale]
                    try:
                        stale.close()
                    except Exception:
                        pass

                self.clients_map[client_sock] = {
                    "ip": rover_ip,
                    "port": rover_port,
                    "connected_at": time.time(),
                    "bytes_sent": 0
                }

            self._add_log(f"Stream Active: Broadcasting RTCM3 to Rover {rover_ip}")

        except Exception as e:
            self._add_log(f"Handshake error with {addr_str}: {e}", "WARN")
            try:
                client_sock.close()
            except Exception:
                pass

    def _parse_nmea_coordinate(self, raw_coord: str, direction: str, is_lon: bool = False) -> Optional[float]:
        """Convert NMEA DDMM.MMMM format to decimal degrees."""
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
        now = time.time()
        if self.survey_start_time is None:
            self.survey_start_time = now
            self._add_log("🛰️ GPS Lock acquired! Starting Survey-In convergence timer (Target: 300s)...")

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

            if self.survey_duration >= self.survey_target_duration and self.survey_accuracy <= self.survey_target_accuracy:
                if not self.survey_valid:
                    self.survey_valid = True
                    self.survey_status = "COMPLETED"
                    self._add_log(f"🎯 Survey-In COMPLETED! Base locked at accuracy: {self.survey_accuracy:.2f}m")
            else:
                self.survey_status = "IN_PROGRESS"

    def _parse_survey_line(self, line: str) -> None:
        """Parses Quectel LC29H Survey-In sentences and standard NMEA sentences."""
        try:
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) >= 10:
                    lat = self._parse_nmea_coordinate(parts[2], parts[3], False)
                    lon = self._parse_nmea_coordinate(parts[4], parts[5], True)
                    if lat and lon:
                        self.survey_lat = lat
                        self.survey_lon = lon
                    if parts[7].isdigit():
                        self.satellites_tracked = int(parts[7])
                    if parts[8].replace('.', '', 1).isdigit():
                        self.hdop = float(parts[8])
                    if parts[9].replace('.', '', 1).isdigit():
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
        """Prints live caster throughput and Survey-In progress to console."""
        while self.is_running:
            time.sleep(6.0)
            with self.clients_lock:
                rover_count = len(self.clients_map)
            rtcm_kb = self.total_rtcm_bytes_read / 1024.0

            remaining_sec = max(0, self.survey_target_duration - self.survey_duration)
            rem_min = remaining_sec // 60
            rem_s = remaining_sec % 60

            if self.survey_valid or self.survey_status == "COMPLETED":
                status_icon = "🎯 [BASE READY]"
                details = f"LOCKED (Accuracy: < {self.survey_accuracy:.2f}m) | Pos: ({self.survey_lat:.7f}, {self.survey_lon:.7f})"
            elif self.survey_duration > 0:
                status_icon = "⏳ [SURVEY-IN]"
                details = f"{self.survey_duration}s/300s ({rem_min}m {rem_s:02d}s left) | Est. Acc: {self.survey_accuracy:.2f}m | Sats: {self.satellites_tracked}"
            else:
                status_icon = "🛰️ [SURVEY-IN]"
                details = f"Tracking {self.satellites_tracked} Sats (HDOP: {self.hdop:.2f}) | Pos: ({self.survey_lat:.7f}, {self.survey_lon:.7f})"

            msg = f"{status_icon} Status: {details} | Active Rovers: {rover_count} | RTCM: {rtcm_kb:.1f} KB"
            print(msg)
            self._add_log(msg)

    def get_status_json(self) -> dict:
        """Returns JSON representation of all base station metrics and live log history."""
        with self.clients_lock:
            active_rovers = [
                {
                    "ip": meta["ip"],
                    "port": meta["port"],
                    "duration_sec": int(time.time() - meta["connected_at"]),
                    "bytes_sent_kb": round(meta["bytes_sent"] / 1024.0, 1)
                }
                for meta in self.clients_map.values()
            ]

        with self.logs_lock:
            log_list = list(self.logs)

        uptime_sec = int(time.time() - self.start_time)
        remaining_sec = max(0, self.survey_target_duration - self.survey_duration)
        rem_min = remaining_sec // 60
        rem_s = remaining_sec % 60
        remaining_str = "0m 00s" if remaining_sec == 0 else f"{rem_min}m {rem_s:02d}s"

        return {
            "uptime_sec": uptime_sec,
            "survey_status": self.survey_status,
            "survey_valid": self.survey_valid or (self.survey_status == "COMPLETED"),
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
                """Prevent reverse DNS lookup that causes 30-60 second delays on local networks."""
                return str(self.client_address[0])

            def log_message(self, format, *args):
                pass

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
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; }
    body { background: var(--bg); color: var(--text); min-height: 100vh; padding: 24px 16px; display: flex; flex-direction: column; align-items: center; }
    .container { width: 100%; max-width: 1000px; }
    
    header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 24px; padding-bottom: 16px; border-bottom: 1px solid var(--card-border); flex-wrap: wrap; gap: 12px; }
    .logo { display: flex; align-items: center; gap: 12px; }
    .logo-icon { font-size: 32px; filter: drop-shadow(0 0 10px var(--accent)); }
    h1 { font-size: 24px; font-weight: 800; background: linear-gradient(135deg, #fff, var(--accent)); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
    .status-badge { display: inline-flex; align-items: center; gap: 8px; padding: 6px 14px; border-radius: 9999px; font-size: 13px; font-weight: 700; letter-spacing: 0.5px; text-transform: uppercase; background: rgba(56, 189, 248, 0.1); border: 1px solid var(--accent); color: var(--accent); }
    .status-badge.locked { background: rgba(16, 185, 129, 0.15); border-color: var(--success); color: var(--success); box-shadow: 0 0 15px var(--success-glow); }
    .pulse-dot { width: 8px; height: 8px; border-radius: 50%; background: currentColor; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0% { transform: scale(0.95); opacity: 0.8; } 50% { transform: scale(1.4); opacity: 1; } 100% { transform: scale(0.95); opacity: 0.8; } }

    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-bottom: 20px; }
    .card { background: var(--card-bg); backdrop-filter: blur(12px); border: 1px solid var(--card-border); border-radius: 16px; padding: 20px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4); }
    .card-title { font-size: 14px; font-weight: 600; text-transform: uppercase; letter-spacing: 1px; color: var(--text-muted); margin-bottom: 16px; display: flex; align-items: center; justify-content: space-between; }
    
    .metric-value { font-size: 32px; font-weight: 800; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; color: #fff; }
    .metric-unit { font-size: 16px; font-weight: 500; color: var(--text-muted); margin-left: 4px; }
    .progress-bar-bg { background: rgba(255, 255, 255, 0.08); height: 10px; border-radius: 6px; overflow: hidden; margin: 12px 0 8px; }
    .progress-bar-fill { background: linear-gradient(90deg, var(--warning), var(--accent)); height: 100%; width: 0%; transition: width 0.5s ease; }
    .progress-bar-fill.complete { background: var(--success); box-shadow: 0 0 12px var(--success-glow); }
    
    .data-row { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid rgba(255, 255, 255, 0.05); font-size: 14px; }
    .data-row:last-child { border-bottom: none; }
    .data-label { color: var(--text-muted); }
    .data-val { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; font-weight: 600; }
    .data-val.countdown { color: #38bdf8; font-weight: 700; }
    
    .console-card { grid-column: 1 / -1; }
    .terminal-box { background: #050811; border: 1px solid rgba(255, 255, 255, 0.1); border-radius: 12px; padding: 14px; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; font-size: 12px; color: #38bdf8; height: 220px; overflow-y: auto; display: flex; flex-direction: column; gap: 4px; }
    .log-line { display: flex; gap: 10px; line-height: 1.4; word-break: break-all; }
    .log-time { color: var(--text-muted); opacity: 0.7; }
    .log-msg { color: #f1f5f9; }
    .log-msg.error { color: #f87171; }
    .log-msg.warn { color: #fbbf24; }

    .code-box { background: rgba(0, 0, 0, 0.4); border: 1px solid rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 12px; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; font-size: 12px; color: #a5f3fc; overflow-x: auto; margin-top: 8px; }
    .btn { display: inline-flex; align-items: center; justify-content: center; gap: 8px; background: var(--accent); color: #000; padding: 8px 16px; border-radius: 8px; font-weight: 700; text-decoration: none; font-size: 13px; margin-top: 12px; transition: transform 0.2s, box-shadow 0.2s; border: none; cursor: pointer; }
    .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 16px var(--accent-glow); }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div class="logo">
        <div class="logo-icon">📡</div>
        <div>
          <h1>RTK Base Station</h1>
          <div style="font-size: 12px; color: var(--text-muted);">Raspberry Pi 5 Local Caster</div>
        </div>
      </div>
      <div id="statusBadge" class="status-badge">
        <div class="pulse-dot"></div>
        <span id="statusText">Surveying...</span>
      </div>
    </header>

    <div class="grid">
      <div class="card">
        <div class="card-title">
          <span>🎯 Survey-In Calibration</span>
          <span id="surveyPercent" class="data-val">0%</span>
        </div>
        <div class="progress-bar-bg">
          <div id="surveyProgressBar" class="progress-bar-fill"></div>
        </div>
        <div class="data-row">
          <span class="data-label">⏳ Time Remaining:</span>
          <span id="surveyRemaining" class="data-val countdown">5m 00s</span>
        </div>
        <div class="data-row">
          <span class="data-label">Elapsed Duration:</span>
          <span id="surveyTime" class="data-val">0s / 300s</span>
        </div>
        <div class="data-row">
          <span class="data-label">Live Accuracy StdDev (σ):</span>
          <span id="surveyAcc" class="data-val">-- m</span>
        </div>
        <div class="data-row">
          <span class="data-label">Target Accuracy Threshold:</span>
          <span class="data-val">&lt; 2.00 m</span>
        </div>
        <div class="data-row">
          <span class="data-label">Anchor Reference Status:</span>
          <span id="anchorStatus" class="data-val" style="color: var(--warning);">Converging...</span>
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
          <span class="data-val">GPS + GLO + GAL + BDS</span>
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
        <a id="mapsBtn" href="#" target="_blank" class="btn">🗺️ Open in Google Maps</a>
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
      <div class="card console-card">
        <div class="card-title">
          <span>🖥️ Live Base Station Console & NMEA Logs</span>
          <span class="data-val" style="font-size: 11px; opacity: 0.7;">Auto-refreshing</span>
        </div>
        <div id="terminalBox" class="terminal-box">
          <div class="log-line"><span class="log-time">[${l.time}]</span><span class="log-msg">Connecting to live log stream...</span></div>
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

    async function updateDashboard() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();

        const isComplete = data.survey_valid || data.survey_status === 'COMPLETED';
        const badge = document.getElementById('statusBadge');
        const statusText = document.getElementById('statusText');
        const progressBar = document.getElementById('surveyProgressBar');
        const anchorStatus = document.getElementById('anchorStatus');
        const remainingEl = document.getElementById('surveyRemaining');

        if (isComplete) {
          badge.className = 'status-badge locked';
          statusText.textContent = '🎯 Base Ready (Locked)';
          progressBar.className = 'progress-bar-fill complete';
          progressBar.style.width = '100%';
          document.getElementById('surveyPercent').textContent = '100%';
          anchorStatus.textContent = 'LOCKED & VALID';
          anchorStatus.style.color = 'var(--success)';
          remainingEl.textContent = '✅ Calibration Complete';
          remainingEl.style.color = 'var(--success)';
        } else {
          badge.className = 'status-badge';
          statusText.textContent = `⏳ Surveying (${data.remaining_str} left)`;
          progressBar.className = 'progress-bar-fill';
          const pct = Math.min(100, Math.round((data.survey_duration / data.survey_target_duration) * 100));
          progressBar.style.width = pct + '%';
          document.getElementById('surveyPercent').textContent = pct + '%';
          anchorStatus.textContent = 'Converging Samples...';
          anchorStatus.style.color = 'var(--warning)';
          remainingEl.textContent = `${data.remaining_str} remaining`;
          remainingEl.style.color = '#38bdf8';
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
    args = parser.parse_args()

    caster = NTRIPBaseCaster(
        serial_port=args.serial,
        baud_rate=args.baud,
        server_port=args.port,
        web_port=args.web_port,
        mountpoint=args.mountpoint,
        password=args.password
    )
    try:
        caster.start()
    except KeyboardInterrupt:
        print("\nStopping NTRIP Base Caster...")
        caster.is_running = False
        sys.exit(0)


if __name__ == '__main__':
    main()
