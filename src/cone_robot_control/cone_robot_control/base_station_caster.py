#!/usr/bin/env python3
"""
==============================================================================
Raspberry Pi 5 RTK Base Station NTRIP Caster Node
==============================================================================
Description:
    Reads raw RTCM3 differential correction packets from the Base GNSS HAT
    (e.g., Waveshare LC29H(BS) / LC29H(EA) on /dev/ttyAMA0 @ 115200 baud)
    and hosts a local NTRIP Caster TCP server on port 2101.

    Allows rovers (Cone Robot) to connect over Wi-Fi and receive live
    centimeter-grade RTK correction streams.

Usage:
    ros2 run cone_robot_control base_station_caster
    or
    python3 base_station_caster.py --port 2101 --mountpoint BASE
==============================================================================
"""

import argparse
import socket
import sys
import threading
import time
from typing import List


class NTRIPBaseCaster:
    def __init__(
        self,
        serial_port: str = "/dev/ttyAMA0",
        baud_rate: int = 115200,
        server_port: int = 2101,
        mountpoint: str = "BASE",
        password: str = "none"
    ) -> None:
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.server_port = server_port
        self.mountpoint = mountpoint.strip("/")
        self.password = password

        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.is_running = True
        self.total_bytes_sent = 0
        self.total_rtcm_bytes_read = 0

    def start(self) -> None:
        """Starts the serial reader and TCP server threads."""
        print("=" * 65)
        print("  📡 RASPBERRY PI 5 RTK BASE STATION NTRIP CASTER")
        print("=" * 65)
        print(f"  • Serial Port       : {self.serial_port} @ {self.baud_rate} baud")
        print(f"  • NTRIP Server Port : {self.server_port}")
        print(f"  • Mountpoint        : /{self.mountpoint}")
        print(f"  • Connection URL    : http://<BASE_PI_IP>:{self.server_port}/{self.mountpoint}")
        print("=" * 65 + "\n")

        # Start background TCP Server thread
        server_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        server_thread.start()

        # Start periodic diagnostics logger
        diag_thread = threading.Thread(target=self._diagnostic_logger_loop, daemon=True)
        diag_thread.start()

        # Run serial reader loop in main thread
        self._serial_reader_loop()

    def _tcp_server_loop(self) -> None:
        """Listens for incoming NTRIP rover client TCP connections."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server_sock.bind(('0.0.0.0', self.server_port))
            server_sock.listen(10)
            print(f"[NTRIP Server] Listening for rovers on port {self.server_port}...")

            while self.is_running:
                client_sock, client_addr = server_sock.accept()
                client_thread = threading.Thread(
                    target=self._handle_client_handshake,
                    args=(client_sock, client_addr),
                    daemon=True
                )
                client_thread.start()
        except Exception as e:
            print(f"❌ [Server Error] {e}")
        finally:
            server_sock.close()

    def _handle_client_handshake(self, client_sock: socket.socket, client_addr: tuple) -> None:
        """Handles standard NTRIP 1.0/2.0 HTTP header handshake with the rover."""
        client_sock.settimeout(5.0)
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
            print(f"[Rover Connected] {client_addr[0]}:{client_addr[1]} -> Request: {first_line}")

            # Verify requested mountpoint
            if f"/{self.mountpoint}" not in first_line and f"/{self.mountpoint.lower()}" not in first_line:
                print(f"⚠️ [Rejected] Rover requested unknown mountpoint: {first_line}")
                client_sock.sendall(b"HTTP/1.0 404 Not Found\r\n\r\n")
                client_sock.close()
                return

            # Respond with standard NTRIP ICY 200 OK
            client_sock.sendall(b"ICY 200 OK\r\n\r\n")
            client_sock.setblocking(False)

            with self.clients_lock:
                self.clients.append(client_sock)
            print(f"✅ [Stream Active] Streaming RTCM3 to Rover: {client_addr[0]} (Active Rovers: {len(self.clients)})")

        except Exception as e:
            print(f"⚠️ [Handshake Error with {client_addr[0]}]: {e}")
            try:
                client_sock.close()
            except Exception:
                pass

    def _serial_reader_loop(self) -> None:
        """Reads raw RTCM3 binary packets from Base GNSS module and multicasts to all rovers."""
        import serial

        while self.is_running:
            ser = None
            try:
                print(f"[Serial] Opening Base GNSS UART: {self.serial_port} @ {self.baud_rate} baud...")
                ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
                print(f"✅ [Serial] Base GNSS UART active! Streaming RTCM3 packets...")

                while self.is_running:
                    # Read incoming RTCM3 binary chunk from base hardware
                    chunk = ser.read(1024)
                    if not chunk:
                        continue

                    self.total_rtcm_bytes_read += len(chunk)

                    # Multicast to all connected rovers
                    with self.clients_lock:
                        dead_clients = []
                        for client in self.clients:
                            try:
                                client.sendall(chunk)
                                self.total_bytes_sent += len(chunk)
                            except (BlockingIOError, socket.error):
                                dead_clients.append(client)

                        for dead in dead_clients:
                            self.clients.remove(dead)
                            try:
                                dead.close()
                            except Exception:
                                pass
                            print(f"ℹ️ [Rover Disconnected] Remaining Active Rovers: {len(self.clients)}")

            except Exception as e:
                print(f"❌ [Serial Error] {e}. Retrying in 2 seconds...")
                time.sleep(2.0)
            finally:
                if ser and ser.is_open:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _diagnostic_logger_loop(self) -> None:
        """Prints live caster throughput status every 10 seconds."""
        while self.is_running:
            time.sleep(10.0)
            with self.clients_lock:
                rover_count = len(self.clients)
            rtcm_kb = self.total_rtcm_bytes_read / 1024.0
            out_kb = self.total_bytes_sent / 1024.0
            print(f"[Base Caster Status] Active Rovers: {rover_count} | Ingested RTCM: {rtcm_kb:.1f} KB | Broadcasted: {out_kb:.1f} KB")


def main() -> None:
    parser = argparse.ArgumentParser(description="Raspberry Pi 5 RTK Base Station NTRIP Caster")
    parser.add_argument('--serial', type=str, default='/dev/ttyAMA0', help="Base GNSS UART port (default: /dev/ttyAMA0)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--port', type=int, default=2101, help="NTRIP server port (default: 2101)")
    parser.add_argument('--mountpoint', type=str, default='BASE', help="NTRIP mountpoint name (default: BASE)")
    parser.add_argument('--password', type=str, default='none', help="Optional authentication password")
    args = parser.parse_args()

    caster = NTRIPBaseCaster(
        serial_port=args.serial,
        baud_rate=args.baud,
        server_port=args.port,
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
