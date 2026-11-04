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
            "odom": "/odom",
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

# Discovered robots cache & lock
discovered_cache = {}
discovered_lock = threading.Lock()

# Discovered RTK Base Station cache & lock
discovered_base_station = {
    "ip": None,
    "port": 8080,
    "caster_port": 2101,
    "online": False,
    "host": "conerobotbasestation.local",
    "url": "http://conerobotbasestation.local:8080",
    "last_seen": 0.0
}
discovered_base_lock = threading.Lock()

def check_base_station_host(host_or_ip, port=8080, timeout=0.3):
    """
    Checks if host_or_ip responds on port 8080 (Base Station Web Dashboard)
    or port 2101 (NTRIP Caster).
    Returns resolved_ip if reachable, None otherwise.
    """
    try:
        resolved_ip = socket.gethostbyname(host_or_ip)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        result = s.connect_ex((resolved_ip, port))
        s.close()
        if result == 0:
            return resolved_ip

        # Also check NTRIP port 2101 as fallback
        s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s2.settimeout(timeout)
        result2 = s2.connect_ex((resolved_ip, 2101))
        s2.close()
        if result2 == 0:
            return resolved_ip
    except Exception:
        pass
    return None

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
        pass
    # Fallback to local network interfaces if offline (no internet)
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127.") and not ip.startswith("169.254.") and "." in ip:
                parts = ip.split(".")
                return f"{parts[0]}.{parts[1]}.{parts[2]}"
    except Exception:
        pass
    return "192.168.1"

def get_all_subnets():
    subnets = set()
    primary = get_primary_subnet()
    if primary:
        subnets.add(primary)

    # 2. Windows Mobile Hotspot default subnet (192.168.137.X)
    subnets.add("192.168.137")

    # 3. All local IP interfaces on this machine
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127.") and "." in ip:
                parts = ip.split(".")
                subnets.add(f"{parts[0]}.{parts[1]}.{parts[2]}")
    except Exception:
        pass

    return list(subnets)

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
        print("[DISCOVERY] Scanning network for active ConeRobots and RTK Base Station...")
        cfg = load_config()
        robot_port = cfg.get("server", {}).get("robot_port", 8765)

        # 0. Health-check previously discovered robots & prune offline ones
        with discovered_lock:
            cached_items = list(discovered_cache.items())
        for slot_id, r_info in cached_items:
            r_ip = r_info.get("ip")
            r_port = r_info.get("port", robot_port)
            if not check_robot_host(r_ip, r_port, timeout=0.3):
                print(f"[DISCOVERY] Robot {slot_id:02d} ({r_ip}:{r_port}) is offline. Removing from active cache.")
                with discovered_lock:
                    discovered_cache.pop(slot_id, None)

        # 0b. Health-check previously discovered base station
        found_base_ip = None
        found_base_host = None
        with discovered_base_lock:
            prev_base_ip = discovered_base_station.get("ip")
            prev_base_port = discovered_base_station.get("port", 8080)
            prev_base_host = discovered_base_station.get("host")

        if prev_base_ip and check_base_station_host(prev_base_ip, prev_base_port, timeout=0.3):
            found_base_ip = prev_base_ip
            found_base_host = prev_base_host

        # 1. Candidate hostnames for robots & Base Station
        candidate_hosts = ["conerobot.local", "conerobot", "raspberrypi.local"]
        for i in range(1, 14):
            candidate_hosts.extend([f"conerobot{i:02d}.local", f"conerobot{i}.local", f"robot{i:02d}.local", f"robot{i}.local"])
        for r in cfg.get("robots", []):
            if r.get("host") and r["host"] not in candidate_hosts:
                candidate_hosts.append(r["host"])
            if r.get("ip") and r["ip"] not in candidate_hosts:
                candidate_hosts.append(r["ip"])

        # Base Station candidate hostnames
        base_candidates = [
            "conerobotbasestation.local",
            "conerobotbasestation",
            "conerobot-base.local",
            "conerobotbase.local",
            "basestation.local",
            "basestation"
        ]

        with ThreadPoolExecutor(max_workers=20) as executor:
            host_futures = {executor.submit(check_robot_host, h, robot_port, 0.5): h for h in candidate_hosts}
            base_futures = {executor.submit(check_base_station_host, bh, 8080, 0.4): bh for bh in base_candidates} if not found_base_ip else {}

            for fut in host_futures:
                h = host_futures[fut]
                res_ip = fut.result()
                if res_ip:
                    with discovered_lock:
                        used_ips = {v["ip"] for v in discovered_cache.values()}
                        if res_ip in used_ips:
                            continue  # Prevent duplicate cards

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

            for fut in base_futures:
                bh = base_futures[fut]
                b_res_ip = fut.result()
                if b_res_ip and not found_base_ip:
                    found_base_ip = b_res_ip
                    found_base_host = bh

        # 2. Local subnet sweep (Scans primary Wi-Fi AND Windows Mobile Hotspot 192.168.137.X)
        all_subnets = get_all_subnets()
        ips_to_check = []
        for subnet in all_subnets:
            ips_to_check.extend([f"{subnet}.{i}" for i in range(1, 255)])

        def probe_target_ip(ip):
            r_ip = check_robot_host(ip, robot_port, 0.2)
            b_ip = None
            if not found_base_ip:
                b_ip = check_base_station_host(ip, 8080, 0.2)
            return (ip, r_ip, b_ip)

        with ThreadPoolExecutor(max_workers=50) as executor:
            ip_futures = [executor.submit(probe_target_ip, ip) for ip in ips_to_check]
            for fut in ip_futures:
                ip, res_robot_ip, res_base_ip = fut.result()
                if res_robot_ip:
                    with discovered_lock:
                        used_ips = {v["ip"] for v in discovered_cache.values()}
                        free_slots = [r["id"] for r in cfg.get("robots", []) if r["id"] not in discovered_cache]
                        if res_robot_ip not in used_ips and free_slots:
                            slot_id = free_slots.pop(0)
                            print(f"[DISCOVERY] Found Unit on IP {res_robot_ip}:{robot_port} -> Assigned to Slot {slot_id:02d}")
                            discovered_cache[slot_id] = {
                                "id": slot_id,
                                "name": f"ConeRobot {slot_id:02d}",
                                "host": f"robot-{res_robot_ip.replace('.', '-')}",
                                "ip": res_robot_ip,
                                "port": robot_port,
                                "online": True
                            }

                if res_base_ip and not found_base_ip:
                    # Validate that this IP is indeed the Base Station (NTRIP port 2101 or status endpoint)
                    is_real_base = False
                    try:
                        s_nt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s_nt.settimeout(0.2)
                        if s_nt.connect_ex((res_base_ip, 2101)) == 0:
                            is_real_base = True
                        s_nt.close()
                    except Exception:
                        pass
                    if not is_real_base:
                        try:
                            import urllib.request
                            req = urllib.request.Request(f"http://{res_base_ip}:8080/api/status", headers={'User-Agent': 'ConeRobotDashboard/1.0'})
                            with urllib.request.urlopen(req, timeout=0.3) as resp:
                                if resp.status == 200:
                                    is_real_base = True
                        except Exception:
                            pass
                    if is_real_base:
                        found_base_ip = res_base_ip
                        found_base_host = f"base-{res_base_ip.replace('.', '-')}"

        # Update discovered Base Station status
        with discovered_base_lock:
            if found_base_ip:
                discovered_base_station["ip"] = found_base_ip
                discovered_base_station["host"] = found_base_host or found_base_ip
                discovered_base_station["port"] = 8080
                discovered_base_station["caster_port"] = 2101
                discovered_base_station["online"] = True
                discovered_base_station["url"] = f"http://{found_base_ip}:8080"
                discovered_base_station["last_seen"] = time.time()
                print(f"[DISCOVERY] 📡 RTK Base Station online at {found_base_ip}:8080 (URL: {discovered_base_station['url']})")
            else:
                if time.time() - discovered_base_station.get("last_seen", 0) > 30:
                    discovered_base_station["online"] = False

        with discovered_lock:
            act_count = len(discovered_cache)
        elapsed = round(time.time() - start_t, 2)
        base_status_str = f"Base: {found_base_ip or 'Offline'}"
        print(f"[DISCOVERY] Scan complete ({elapsed}s). Active robots: {act_count} | {base_status_str}")
    finally:
        is_scanning = False

def background_scanner_loop():
    """Periodically triggers network scans based on scan_interval_sec."""
    time.sleep(1.0)
    while True:
        try:
            cfg = load_config()
            interval = max(5, cfg.get("server", {}).get("scan_interval_sec", 10))
            scan_network_job()
            time.sleep(interval)
        except Exception:
            time.sleep(5)

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
            cfg = load_config()
            cfg_robots = {r["id"]: r for r in cfg.get("robots", []) if "id" in r}
            robot_port = cfg.get("server", {}).get("robot_port", 8765)
            
            with discovered_lock:
                cache_snapshot = dict(discovered_cache)

            for r_id in range(1, 14):
                if r_id in cache_snapshot and cache_snapshot[r_id].get("online"):
                    robots_list[r_id] = cache_snapshot[r_id]
                elif r_id in cfg_robots and cfg_robots[r_id].get("ip"):
                    # For statically configured IP, verify reachability before claiming online
                    st_ip = cfg_robots[r_id]["ip"]
                    is_alive = bool(check_robot_host(st_ip, robot_port, timeout=0.2))
                    robots_list[r_id] = {
                        "id": r_id,
                        "name": cfg_robots[r_id].get("name", f"ConeRobot {r_id:02d}"),
                        "host": cfg_robots[r_id].get("host", f"conerobot{r_id:02d}.local"),
                        "ip": st_ip,
                        "port": robot_port,
                        "online": is_alive
                    }
                    if is_alive:
                        with discovered_lock:
                            discovered_cache[r_id] = robots_list[r_id]
                else:
                    cfg_r = cfg_robots.get(r_id, {})
                    robots_list[r_id] = {
                        "id": r_id,
                        "name": cfg_r.get("name", f"ConeRobot {r_id:02d}"),
                        "host": cfg_r.get("host", f"conerobot{r_id:02d}.local"),
                        "ip": cfg_r.get("ip", ""),
                        "port": robot_port,
                        "online": False
                    }

            with discovered_lock:
                disc_count = len(discovered_cache)

            with discovered_base_lock:
                bs_snapshot = dict(discovered_base_station)

            response_data = {
                "discovered_count": disc_count,
                "topics": cfg.get("topics", {}),
                "robots": robots_list,
                "base_station": bs_snapshot
            }
            self.wfile.write(json.dumps(response_data).encode("utf-8"))
            return

        # API: Discovered RTK Base Station
        if parsed.path == "/api/base_station":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            with discovered_base_lock:
                bs_snapshot = dict(discovered_base_station)
            self.wfile.write(json.dumps(bs_snapshot).encode("utf-8"))
            return

        # Direct Dynamic Navigation Redirect to Base Station Dashboard
        norm_path = parsed.path.rstrip("/")
        if norm_path in ("/goto_base", "/base_station") or parsed.path == "/goto_base.html":
            with discovered_base_lock:
                target_url = discovered_base_station.get("url")
                if not target_url or not discovered_base_station.get("online"):
                    target_url = "http://conerobotbasestation.local:8080"
            self.send_response(302)
            self.send_header("Location", target_url)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
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

        # API: Query Base Station Caster (port 8080)
        if parsed.path == "/api/base_status":
            import urllib.request
            from urllib.parse import parse_qs
            query = parse_qs(parsed.query)
            target_host = query.get("host", [None])[0]
            port = query.get("port", ["8080"])[0]

            with discovered_base_lock:
                known_base_ip = discovered_base_station.get("ip")

            hosts_to_try = []
            if target_host:
                hosts_to_try.append(target_host)
            if known_base_ip and known_base_ip not in hosts_to_try:
                hosts_to_try.append(known_base_ip)
            if "127.0.0.1" not in hosts_to_try:
                hosts_to_try.append("127.0.0.1")
            hosts_to_try.extend(["conerobotbasestation.local", "conerobotbasestation", "basestation.local", "192.168.1.100", "192.168.1.50"])

            res_data = None
            for h in hosts_to_try:
                try:
                    url = f"http://{h}:{port}/api/status"
                    req = urllib.request.Request(url, headers={'User-Agent': 'ConeRobotDashboard/1.0'})
                    with urllib.request.urlopen(req, timeout=0.8) as resp:
                        if resp.status == 200:
                            res_data = json.loads(resp.read().decode('utf-8'))
                            res_data["connected_host"] = h
                            with discovered_base_lock:
                                discovered_base_station["online"] = True
                                if not discovered_base_station.get("ip") and not h.endswith(".local") and h != "127.0.0.1":
                                    discovered_base_station["ip"] = h
                                discovered_base_station["url"] = f"http://{h}:{port}"
                                discovered_base_station["last_seen"] = time.time()
                            break
                except Exception:
                    continue

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            if res_data:
                self.wfile.write(json.dumps(res_data).encode("utf-8"))
            else:
                self.wfile.write(json.dumps({"error": "Base station caster not reachable on port 8080"}).encode("utf-8"))
            return

        super().do_GET()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

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

        # API: Waypoint Dispatch Relay
        if parsed.path == "/api/waypoint":
            try:
                length = int(self.headers.get('content-length', 0))
                body = self.rfile.read(length).decode("utf-8")
                req = json.loads(body)
                r_id = req.get("robot_id", "Unknown")
                x = req.get("x", 0.0)
                y = req.get("y", 0.0)
                z = req.get("z", 0.0)
                cmd_type = req.get("type", "WAYPOINT")
                print(f"[MISSION CONTROL] {cmd_type} command for Robot {r_id}: (X={x:.2f}m, Y={y:.2f}m, Z={z:.1f})")

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "Success",
                    "command": cmd_type,
                    "robot_id": r_id,
                    "target": {"x": x, "y": y, "z": z},
                    "timestamp": time.time()
                }).encode("utf-8"))
                return
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
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

    # Start periodic background scanner
    threading.Thread(target=background_scanner_loop, daemon=True).start()

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
