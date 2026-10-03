#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Node: Waveshare LC29H(DA) Dual-Band GPS/RTK Driver & NTRIP Rover Client
==============================================================================
Description:
    Reads high-precision GNSS NMEA sentences from the Waveshare LC29H(DA) HAT
    over Raspberry Pi 5 hardware UART (/dev/ttyAMA0).
    
    Includes an integrated, auto-reconnecting NTRIP Rover client that connects
    to public casters (e.g., RTK2Go, CORS) or local/private base stations over
    Wi-Fi/Ethernet, streaming RTCM3 differential corrections directly into the
    LC29H module to achieve RTK Float / RTK Fix centimeter accuracy.

Topics:
    - /fix (sensor_msgs/msg/NavSatFix): Standard ROS 2 GPS fix with covariance.
    - /gps/status (std_msgs/msg/String): Human-readable RTK & satellite status.

Author: ConeRobot Team
License: MIT
==============================================================================
"""

import base64
from concurrent.futures import ThreadPoolExecutor
import json
import math
import socket
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class LC29HGPSNode(Node):
    """
    ROS 2 driver for Waveshare LC29H(DA) GPS/RTK HAT with integrated NTRIP client.
    """

    # NMEA Fix Quality Mapping
    FIX_QUALITY_MAP = {
        0: ("NO FIX", NavSatStatus.STATUS_NO_FIX, 10000.0),
        1: ("3D FIX (SPS)", NavSatStatus.STATUS_FIX, 2.5),
        2: ("DGPS FIX", NavSatStatus.STATUS_SBAS_FIX, 1.0),
        4: ("RTK FIX", NavSatStatus.STATUS_GBAS_FIX, 0.02),
        5: ("RTK FLOAT", NavSatStatus.STATUS_GBAS_FIX, 0.20),
        6: ("ESTIMATED", NavSatStatus.STATUS_NO_FIX, 10.0),
    }

    def __init__(self) -> None:
        super().__init__('lc29h_gps_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mock_hardware', False)

        # NTRIP Caster Configuration (Supports Public Casters & Local/Private Base Stations)
        self.declare_parameter('ntrip_enable', True)
        self.declare_parameter('ntrip_caster', 'rtk2go.com')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'PFORZEM')
        self.declare_parameter('ntrip_user', 'conerobot@rover.local')
        self.declare_parameter('ntrip_password', 'none')
        self.declare_parameter('ntrip_send_gga', True)

        # Retrieve Parameters
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.mock_hardware = self.get_parameter('mock_hardware').get_parameter_value().bool_value

        self.ntrip_enable = self.get_parameter('ntrip_enable').get_parameter_value().bool_value
        self.ntrip_caster = self.get_parameter('ntrip_caster').get_parameter_value().string_value
        self.ntrip_port = self.get_parameter('ntrip_port').get_parameter_value().integer_value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').get_parameter_value().string_value
        self.ntrip_user = self.get_parameter('ntrip_user').get_parameter_value().string_value
        self.ntrip_password = self.get_parameter('ntrip_password').get_parameter_value().string_value
        self.ntrip_send_gga = self.get_parameter('ntrip_send_gga').get_parameter_value().bool_value

        # ROS 2 Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.status_pub = self.create_publisher(String, '/gps/status', 10)

        # Internal State
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.is_running = True
        self.latest_gga_raw = ""
        self.ntrip_connected = False
        self.rtcm_bytes_received = 0
        self._cached_base_ip: Optional[str] = None

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.current_fix_quality = 0
        self.current_num_sats = 0
        self.current_hdop = 99.99
        self.last_fix_time = 0.0
        self._last_diag_log_time = 0.0
        self._last_logged_fix_quality = -1

        self.get_logger().info("==================================================")
        self.get_logger().info(" Waveshare LC29H(DA) Dual-Band GPS/RTK Driver")
        self.get_logger().info(f" Serial Port : {self.serial_port_name} @ {self.baud_rate} baud")
        self.get_logger().info(f" NTRIP Client: {'Enabled' if self.ntrip_enable else 'Disabled'}")
        if self.ntrip_enable:
            self.get_logger().info(f" NTRIP Caster: {self.ntrip_caster}:{self.ntrip_port}/{self.ntrip_mountpoint}")
        self.get_logger().info(f" Mock Mode   : {self.mock_hardware}")
        self.get_logger().info("==================================================")

        # Initialize Hardware or Mock
        if not self.mock_hardware:
            self._init_serial()
            # Start background serial read thread
            self.serial_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self.serial_thread.start()

            # Start background NTRIP client thread if enabled
            if self.ntrip_enable:
                self.ntrip_thread = threading.Thread(target=self._ntrip_client_loop, daemon=True)
                self.ntrip_thread.start()
        else:
            self.get_logger().warn("Mock hardware enabled: generating simulated RTK GPS fix data.")
            self.mock_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_mock_data)

        # Periodic status publisher timer (1 Hz for real-time dashboard telemetry)
        self.status_timer = self.create_timer(1.0, self._publish_diagnostic_status)

    def _init_serial(self) -> None:
        """Initialize serial connection to Raspberry Pi 5 UART."""
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.get_logger().info(f"Successfully opened serial port: {self.serial_port_name}")

            # Explicitly ensure LC29H(DA) is configured in Rover RTK mode
            try:
                with self.serial_lock:
                    self.serial_conn.write(b"$PQTMCFGRCVRMODE,W,1*2A\r\n")
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            self.get_logger().error("Ensure user is in dialout group and port permissions are set.")

    def _serial_read_loop(self) -> None:
        """Continuously reads NMEA sentences from the serial port using fast chunk buffering."""
        raw_buffer = bytearray()
        while rclpy.ok() and self.is_running:
            if not self.serial_conn or not self.serial_conn.is_open:
                time.sleep(1.0)
                continue

            try:
                chunk = self.serial_conn.read(1024)
                if not chunk:
                    continue

                raw_buffer.extend(chunk)
                if len(raw_buffer) > 8192:
                    raw_buffer = raw_buffer[-4096:]

                while b'\n' in raw_buffer:
                    line_bytes, _, remaining = raw_buffer.partition(b'\n')
                    raw_buffer = remaining
                    line = line_bytes.decode('ascii', errors='ignore').strip()
                    if '$' in line:
                        clean_sentence = line[line.find('$'):]
                        header = clean_sentence.split(',')[0]
                        if header.endswith('GGA'):
                            self._parse_gga(clean_sentence)
                        elif header.endswith('RMC'):
                            self._parse_rmc(clean_sentence)

            except Exception as e:
                self.get_logger().debug(f"Serial read error: {e}")
                time.sleep(0.05)

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

    def _parse_gga(self, line: str) -> None:
        """Parse NMEA $GNGGA sentence for position, altitude, and RTK fix status."""
        parts = line.split(',')
        if len(parts) < 10:
            return

        self.latest_gga_raw = line

        try:
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]
            fix_qual_str = parts[6] if len(parts) > 6 else "0"
            num_sats_str = parts[7] if len(parts) > 7 else "0"
            hdop_str = parts[8] if len(parts) > 8 else "99.99"
            alt_str = parts[9] if len(parts) > 9 else "0.0"

            if num_sats_str.isdigit() and int(num_sats_str) > 0:
                self.current_num_sats = int(num_sats_str)
            if fix_qual_str.isdigit():
                self.current_fix_quality = int(fix_qual_str)
            try:
                self.current_hdop = float(hdop_str)
            except ValueError:
                self.current_hdop = 99.99
            try:
                self.current_alt = float(alt_str)
            except ValueError:
                self.current_alt = 0.0

            lat = self._parse_nmea_coordinate(raw_lat, lat_dir, is_lon=False)
            lon = self._parse_nmea_coordinate(raw_lon, lon_dir, is_lon=True)

            if lat is not None and lon is not None:
                self.current_lat = lat
                self.current_lon = lon
                self.last_fix_time = time.time()
                self._publish_navsat_fix()

        except Exception as e:
            self.get_logger().debug(f"Error parsing GGA: {e}")

    def _parse_gsv(self, line: str) -> None:
        """Parse NMEA GSV sentences to track satellites in view."""
        try:
            parts = line.split(',')
            if len(parts) >= 4 and parts[3].isdigit():
                sats_in_view = int(parts[3])
                if sats_in_view > self.current_num_sats and self.current_fix_quality == 0:
                    self.current_num_sats = sats_in_view
        except Exception:
            pass

    def _parse_rmc(self, line: str) -> None:
        """Parse NMEA $GNRMC sentence for speed/heading fallback."""
        pass

    def _publish_navsat_fix(self) -> None:
        """Construct and publish a standard sensor_msgs/NavSatFix message."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Map fix quality to ROS NavSatStatus
        quality_info = self.FIX_QUALITY_MAP.get(
            self.current_fix_quality,
            ("UNKNOWN", NavSatStatus.STATUS_NO_FIX, 100.0)
        )
        _, nav_status, base_std_dev = quality_info

        msg.status.status = nav_status
        msg.status.service = (
            NavSatStatus.SERVICE_GPS |
            NavSatStatus.SERVICE_GLONASS |
            NavSatStatus.SERVICE_GALILEO |
            NavSatStatus.SERVICE_COMPASS
        )

        msg.latitude = self.current_lat
        msg.longitude = self.current_lon
        msg.altitude = self.current_alt

        # Calculate position covariance matrix (sigma^2)
        if nav_status != NavSatStatus.STATUS_NO_FIX:
            var_h = (base_std_dev * max(self.current_hdop, 0.5)) ** 2
            var_v = (base_std_dev * 2.0 * max(self.current_hdop, 0.5)) ** 2
            msg.position_covariance = [
                var_h, 0.0, 0.0,
                0.0, var_h, 0.0,
                0.0, 0.0, var_v
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance = [10000.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)

    def _get_local_subnets(self) -> List[str]:
        """
        Discovers all local /24 subnet prefixes (e.g. ['192.168.0.', '192.168.137.'])
        across all active network interfaces (Wi-Fi, Ethernet, Hotspots).
        """
        subnets = set()

        # 1. Linux 'hostname -I' command (reads all active IP assignments)
        try:
            import subprocess
            out = subprocess.check_output(["hostname", "-I"], timeout=1.0).decode('ascii').strip()
            for ip in out.split():
                parts = ip.strip().split('.')
                if len(parts) == 4 and not ip.startswith('127.'):
                    subnets.add(f"{parts[0]}.{parts[1]}.{parts[2]}.")
        except Exception:
            pass

        # 2. Socket routing probe to common gateways
        for test_target in ["8.8.8.8", "192.168.0.1", "192.168.1.1", "192.168.137.1", "10.0.0.1"]:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.connect((test_target, 80))
                    ip = s.getsockname()[0]
                    if ip and not ip.startswith("127."):
                        parts = ip.split('.')
                        if len(parts) == 4:
                            subnets.add(f"{parts[0]}.{parts[1]}.{parts[2]}.")
            except Exception:
                pass

        # 3. Hostname fallback
        try:
            host_ip = socket.gethostbyname(socket.gethostname())
            if host_ip and not host_ip.startswith("127."):
                parts = host_ip.split('.')
                if len(parts) == 4:
                    subnets.add(f"{parts[0]}.{parts[1]}.{parts[2]}.")
        except Exception:
            pass

        return list(subnets)

    def _listen_for_udp_beacon(self, timeout_sec: float = 0.8) -> Optional[str]:
        """
        Listens on UDP port 2102 for RTK Base Station broadcast beacon.
        Instant pairing (< 0.8s) without scanning when Base is broadcasting.
        """
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(timeout_sec)
            sock.bind(('', 2102))

            data, addr = sock.recvfrom(2048)
            payload = json.loads(data.decode('utf-8'))
            if payload.get("service") == "conerobot-rtk-base":
                base_ip = payload.get("ip") or addr[0]
                with socket.create_connection((base_ip, self.ntrip_port), timeout=0.3):
                    return base_ip
        except Exception:
            pass
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
        return None

    def _scan_local_subnet_for_base(self) -> Optional[str]:
        """
        Fast parallel TCP port scanner to automatically locate the Base Station
        across all local subnets (e.g. 192.168.0.x, 192.168.137.x).
        Scans all hosts concurrently using 80 threads.
        """
        subnets = self._get_local_subnets()
        if not subnets:
            return None

        found_ip = None

        def probe(target_ip: str):
            nonlocal found_ip
            if found_ip is not None:
                return
            try:
                with socket.create_connection((target_ip, self.ntrip_port), timeout=0.35):
                    found_ip = target_ip
            except Exception:
                pass

        all_targets = []
        for prefix in subnets:
            for host_num in range(1, 255):
                all_targets.append(f"{prefix}{host_num}")

        with ThreadPoolExecutor(max_workers=80) as executor:
            executor.map(probe, all_targets)

        return found_ip

    def _resolve_caster_host(self) -> Optional[str]:
        """
        Dynamically discovers and resolves the NTRIP Caster IP address.
        Supports:
          1. Explicit static IP (e.g. '192.168.0.105')
          2. Public domains (e.g. 'rtk2go.com')
          3. Cached Base Station IP if still alive
          4. Zero-Config Layer 1: UDP Broadcast Beacon Listener (< 0.8s)
          5. Zero-Config Layer 2: Fast Parallel Subnet TCP Port Probe
          6. Fallback: mDNS local hostname (e.g. 'conerobotBaseStation.local')
        """
        caster_str = self.ntrip_caster.strip()

        # 1. If explicitly set to an IP address, use directly
        try:
            socket.inet_aton(caster_str)
            return caster_str
        except (socket.error, ValueError):
            pass

        # 2. Public domains (e.g. 'rtk2go.com')
        if caster_str.lower() != "auto" and not caster_str.endswith(".local"):
            try:
                ip = socket.gethostbyname(caster_str)
                return ip
            except socket.gaierror:
                pass

        # 3. Check if cached Base Station IP is still reachable
        if self._cached_base_ip:
            try:
                with socket.create_connection((self._cached_base_ip, self.ntrip_port), timeout=0.25):
                    return self._cached_base_ip
            except Exception:
                self._cached_base_ip = None

        # 4. Zero-Config Layer 1: UDP Broadcast Beacon Listener (0.8s)
        beacon_ip = self._listen_for_udp_beacon(timeout_sec=0.8)
        if beacon_ip:
            self._cached_base_ip = beacon_ip
            self.get_logger().info(
                f"[NTRIP Discovery] ✨ Discovered RTK Base Station via UDP beacon at: {beacon_ip}:{self.ntrip_port}!"
            )
            return beacon_ip

        # 5. Zero-Config Layer 2: Fast Parallel Subnet TCP Port Probe
        self.get_logger().info(
            f"[NTRIP Discovery] Probing local network for RTK Base Station (Port {self.ntrip_port})..."
        )
        scanned_ip = self._scan_local_subnet_for_base()
        if scanned_ip:
            self._cached_base_ip = scanned_ip
            self.get_logger().info(
                f"[NTRIP Discovery] ✨ Found RTK Base Station automatically at: {scanned_ip}:{self.ntrip_port}!"
            )
            return scanned_ip

        # 6. Fallback: Try mDNS if hostname was provided and not 'auto'
        if caster_str.lower() != "auto":
            try:
                ip = socket.gethostbyname(caster_str)
                return ip
            except socket.gaierror:
                pass

        # Not found yet
        self.get_logger().warn(
            f"[NTRIP Discovery] 🔍 RTK Base Station not detected on local network. Ensure Base Pi Zero is powered on and connected to Wi-Fi. Retrying..."
        )
        return None

    def _ntrip_client_loop(self) -> None:
        """
        Background NTRIP Rover client loop with auto-reconnect.
        Connects to base stations (public or local), receives RTCM3 correction
        packets, and streams them into the LC29H serial port.
        """
        while rclpy.ok() and self.is_running:
            sock = None
            try:
                # Wait until GPS acquires first NMEA position before connecting to NTRIP
                if not self.latest_gga_raw:
                    self.get_logger().info("[NTRIP] Waiting for initial GPS satellite lock before connecting to caster...")
                    while rclpy.ok() and self.is_running and not self.latest_gga_raw:
                        time.sleep(0.5)

                if not (rclpy.ok() and self.is_running):
                    break

                target_host = self._resolve_caster_host()
                if not target_host:
                    time.sleep(3.0)
                    continue

                self.get_logger().info(
                    f"[NTRIP] Connecting to {target_host}:{self.ntrip_port}/{self.ntrip_mountpoint}..."
                )
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(6.0)
                sock.connect((target_host, self.ntrip_port))

                # Build standard NTRIP Request Header
                auth_str = f"{self.ntrip_user}:{self.ntrip_password}"
                auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
                
                headers = [
                    f"GET /{self.ntrip_mountpoint} HTTP/1.0",
                    "User-Agent: NTRIP ConeRobot/1.0",
                    "Accept: */*",
                    f"Authorization: Basic {auth_b64}",
                ]
                
                if self.ntrip_send_gga and self.latest_gga_raw:
                    headers.append(f"Ntrip-GGA: {self.latest_gga_raw.strip()}")
                
                headers.append("Connection: close\r\n\r\n")
                http_req = "\r\n".join(headers)
                sock.sendall(http_req.encode('ascii'))

                # Read response headers (support both standard \r\n\r\n and Shoutcast/ICY \r\n)
                header_data = b""
                sock.settimeout(6.0)
                while True:
                    chunk = sock.recv(1024)
                    if not chunk:
                        raise ConnectionError("NTRIP Caster closed socket during header handshake.")
                    header_data += chunk
                    if b"ICY 200 OK\r\n" in header_data or b"\r\n\r\n" in header_data:
                        break

                header_text = header_data.decode('latin1', errors='ignore')
                first_line = header_text.splitlines()[0] if header_text else "EMPTY"
                self.get_logger().info(f"[NTRIP] Caster Response: {first_line}")

                if "ICY 200 OK" not in header_text and "200 OK" not in header_text and "HTTP/1.1 200" not in header_text and "HTTP/1.0 200" not in header_text:
                    raise ConnectionError(f"Caster rejected mountpoint [{self.ntrip_mountpoint}]: {first_line}")

                # Extract any binary RTCM3 data that arrived after the header delimiter
                initial_rtcm = b""
                if b"\r\n\r\n" in header_data:
                    _, _, initial_rtcm = header_data.partition(b"\r\n\r\n")
                elif b"ICY 200 OK\r\n" in header_data:
                    _, _, initial_rtcm = header_data.partition(b"ICY 200 OK\r\n")

                if initial_rtcm:
                    self.rtcm_bytes_received += len(initial_rtcm)
                    if self.serial_conn and self.serial_conn.is_open:
                        with self.serial_lock:
                            self.serial_conn.write(initial_rtcm)

                self.get_logger().info(
                    f"[NTRIP] Stream Connected! Receiving live RTCM3 corrections from [{self.ntrip_mountpoint}]"
                )
                self.ntrip_connected = True
                sock.settimeout(30.0)
                last_gga_send_time = time.time()

                # Stream RTCM3 binary correction data to LC29H serial port
                while rclpy.ok() and self.is_running:
                    # Periodically send GGA feedback position back to caster (every 10s for VRS / keepalive)
                    if self.ntrip_send_gga and (time.time() - last_gga_send_time > 10.0):
                        if self.latest_gga_raw:
                            sock.sendall((self.latest_gga_raw.strip() + "\r\n").encode('ascii'))
                        last_gga_send_time = time.time()

                    # Receive binary RTCM3 correction packet
                    rtcm_data = sock.recv(2048)
                    if not rtcm_data:
                        raise ConnectionError("NTRIP socket returned 0 bytes (connection closed by server).")

                    self.rtcm_bytes_received += len(rtcm_data)

                    # Write RTCM3 binary bytes directly into the LC29H HAT UART
                    if self.serial_conn and self.serial_conn.is_open:
                        with self.serial_lock:
                            self.serial_conn.write(rtcm_data)

            except Exception as e:
                self.ntrip_connected = False
                self.get_logger().error(f"[NTRIP Error] {e}. Retrying in 3 seconds...")
                time.sleep(3.0)

            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def _publish_diagnostic_status(self) -> None:
        """Publishes human-readable status string and logs periodic diagnostics."""
        quality_str, _, _ = self.FIX_QUALITY_MAP.get(
            self.current_fix_quality, ("UNKNOWN", NavSatStatus.STATUS_NO_FIX, 100.0)
        )
        
        status_msg = String()
        status_text = (
            f"Fix: {quality_str} | Sats: {self.current_num_sats} | HDOP: {self.current_hdop:.2f} | "
            f"NTRIP: {'Connected' if self.ntrip_connected else ('Disabled' if not self.ntrip_enable else 'Connecting...')} "
            f"({self.rtcm_bytes_received / 1024.0:.1f} KB RTCM)"
        )
        status_msg.data = status_text
        self.status_pub.publish(status_msg)

        # Console log throttled to every 5 seconds (or immediately if fix quality changes)
        now = time.time()
        should_log = (now - self._last_diag_log_time >= 5.0) or (self.current_fix_quality != self._last_logged_fix_quality)
        if should_log:
            self._last_diag_log_time = now
            self._last_logged_fix_quality = self.current_fix_quality
            if self.current_fix_quality in [4, 5]:
                self.get_logger().info(f"[RTK ACTIVE] {status_text} | Pos: ({self.current_lat:.7f}, {self.current_lon:.7f})")
            elif self.current_fix_quality > 0:
                self.get_logger().info(f"[GNSS 3D] {status_text} | Pos: ({self.current_lat:.7f}, {self.current_lon:.7f})")
            else:
                self.get_logger().warn(f"[SEARCHING SATELLITES] {status_text}")

    def _publish_mock_data(self) -> None:
        """Simulate realistic RTK Float / RTK Fix data in mock mode."""
        self.current_lat = 49.0054911 + 0.000005 * math.sin(time.time() * 0.2)
        self.current_lon = 8.2457705 + 0.000005 * math.cos(time.time() * 0.2)
        self.current_alt = 135.0
        self.current_fix_quality = 5  # RTK Float
        self.current_num_sats = 35
        self.current_hdop = 0.43
        self.ntrip_connected = True
        self._publish_navsat_fix()

    def destroy_node(self) -> None:
        """Clean up serial connection on node shutdown."""
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LC29HGPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
