#!/usr/bin/env python3
"""
ConeRobot Multi-Robot Fleet Monitoring Server
============================================
Lightweight Python server that runs on the Laptop.
- 0 MB RAM load on Raspberry Pis.
- Discovers active robots on local network (Port 8765 / .local hostnames).
- Serves the dashboard frontend with direct WebSocket connection to real robots.
- 100% Real Live Sensor Data - Zero Fake / Simulated Data.
"""

import os
import sys
import json
import time
import socket
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse
from concurrent.futures import ThreadPoolExecutor

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "fleet_config.json")
STATIC_DIR = os.path.join(BASE_DIR, "static")

def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[WARN] Error loading config: {e}")
    return {
        "server": {"port": 8000, "host": "0.0.0.0", "robot_port": 8765},
        "topics": {
            "scan": "/scan",
            "gps_fix": "/fix",
            "gps_status": "/gps/status",
            "heading": "/imu/heading",
            "imu_data": "/imu/data",
            "step_status": "/step_status",
            "battery": "/battery_state"
        },
        "robots": [{"id": i, "name": f"ConeRobot {i:02d}", "host": f"conerobot{i:02d}.local", "ip": ""} for i in range(1, 14)],
        "map": {"default_lat": 47.4979, "default_lon": 19.0402, "default_zoom": 18}
    }

def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        return True
    except Exception as e:
        print(f"[ERROR] Failed to save config: {e}")
        return False

# Discovered robots cache
discovered_cache = {}

def get_primary_subnet():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        parts = local_ip.split(".")
        return f"{parts[0]}.{parts[1]}.{parts[2]}"
    except Exception:
        return "192.168.0"

def check_robot_host(host_or_ip, port=8765, timeout=0.5):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        resolved_ip = socket.gethostbyname(host_or_ip)
        result = s.connect_ex((resolved_ip, port))
        s.close()
        if result == 0:
            return resolved_ip
    except Exception:
        pass
    return None

is_scanning = False

def scan_network_job():
    global is_scanning
    if is_scanning:
        return
    is_scanning = True
    start_t = time.time()
    
    try:
        print("[DISCOVERY] Scanning network for active ConeRobots on Port 8765...")
        cfg = load_config()
        robot_port = cfg.get("server", {}).get("robot_port", 8765)

        # 1. Candidate hostnames
        candidate_hosts = ["conerobot.local", "conerobot", "raspberrypi.local"]
        for i in range(1, 14):
            candidate_hosts.extend([f"conerobot{i:02d}.local", f"conerobot{i}.local", f"robot{i:02d}.local", f"robot{i}.local"])

        with ThreadPoolExecutor(max_workers=20) as executor:
            host_futures = {executor.submit(check_robot_host, h, robot_port, 0.5): h for h in candidate_hosts}
            for fut in host_futures:
                h = host_futures[fut]
                res_ip = fut.result()
                if res_ip:
                    used_ips = {v["ip"] for v in discovered_cache.values()}
                    if res_ip in used_ips:
                        continue  # Prevent adding duplicate cards for the same physical robot

                    # Extract numeric robot ID from hostname if present
                    import re
                    match = re.search(r'(\d+)', h)
                    if match and 1 <= int(match.group(1)) <= 13:
                        slot_id = int(match.group(1))
                    else:
                        free_slots = [r["id"] for r in cfg.get("robots", []) if r["id"] not in discovered_cache]
                        slot_id = free_slots[0] if free_slots else 1

                    print(f"[DISCOVERY] Found ConeRobot via '{h}' -> {res_ip}:{robot_port} (Assigned to Robot {slot_id:02d})")
                    discovered_cache[slot_id] = {
                        "id": slot_id,
                        "name": f"ConeRobot {slot_id:02d}",
                        "host": h,
                        "ip": res_ip,
                        "port": robot_port,
                        "online": True
                    }

        # 2. Local subnet sweep
        subnet_prefix = get_primary_subnet()
        ips_to_check = [f"{subnet_prefix}.{i}" for i in range(1, 255)]
        
        with ThreadPoolExecutor(max_workers=30) as executor:
            ip_futures = {executor.submit(check_robot_host, ip, robot_port, 0.2): ip for ip in ips_to_check}
            for fut in ip_futures:
                res_ip = fut.result()
                if res_ip:
                    used_ips = {v["ip"] for v in discovered_cache.values()}
                    free_slots = [r["id"] for r in cfg.get("robots", []) if r["id"] not in discovered_cache]
                    if res_ip not in used_ips and free_slots:
                        slot_id = free_slots.pop(0)
                        print(f"[DISCOVERY] Found Unit on IP {res_ip}:{robot_port} -> Assigned to Slot {slot_id:02d}")
                        discovered_cache[slot_id] = {
                            "id": slot_id,
                            "name": f"ConeRobot {slot_id:02d}",
                            "host": f"robot-{res_ip.replace('.', '-')}",
                            "ip": res_ip,
                            "port": robot_port,
                            "online": True
                        }

        elapsed = round(time.time() - start_t, 2)
        print(f"[DISCOVERY] Scan complete ({elapsed}s). Active robots: {len(discovered_cache)}")
    finally:
        is_scanning = False

class FleetDashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        
        # API: Discovered Robots
        if parsed.path == "/api/fleet":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            robots_list = {}
            for r_id in range(1, 14):
                if r_id in discovered_cache:
                    robots_list[r_id] = discovered_cache[r_id]
                else:
                    robots_list[r_id] = {"id": r_id, "name": f"ConeRobot {r_id:02d}", "online": False}

            response_data = {
                "discovered_count": len(discovered_cache),
                "topics": load_config().get("topics", {}),
                "robots": robots_list
            }
            self.wfile.write(json.dumps(response_data).encode("utf-8"))
            return

        # API: Config
        if parsed.path == "/api/config":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(load_config()).encode("utf-8"))
            return

        # API: Manual Scan
        if parsed.path == "/api/scan":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            threading.Thread(target=scan_network_job, daemon=True).start()
            self.wfile.write(json.dumps({"status": "Scan started"}).encode("utf-8"))
            return

        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            try:
                length = int(self.headers.get('content-length', 0))
                body = self.rfile.read(length).decode("utf-8")
                new_cfg = json.loads(body)
                save_config(new_cfg)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "Saved"}).encode("utf-8"))
                return
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
                return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        return

def main():
    cfg = load_config()
    port = cfg.get("server", {}).get("port", 8000)
    host = cfg.get("server", {}).get("host", "0.0.0.0")

    # Initial background scan on startup
    threading.Thread(target=scan_network_job, daemon=True).start()

    print("\n" + "=" * 70)
    print("  🚀 ConeRobot 13-Robot Fleet Monitoring Server")
    local_prefix = get_primary_subnet()
    print(f"  📍 Local Laptop URL : http://localhost:{port}")
    print(f"  📍 Wi-Fi Network URL : http://{local_prefix}.XXX:{port}")
    print("=" * 70)
    print("  • 0 MB RAM load on Raspberry Pis.")
    print("  • Direct Foxglove WebSocket Streaming.")
    print("  • 100% Real Live Sensor Data (Zero Fake / Simulated Data).")
    print("  • Press Ctrl+C in terminal to stop.\n")

    httpd = HTTPServer((host, port), FleetDashboardHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping Fleet Dashboard server...")
        httpd.server_close()
        sys.exit(0)

if __name__ == "__main__":
    main()
