#!/usr/bin/env python3
"""
ConeRobot Protocol Finder Tool
Finds the exact handshake header that Foxglove Bridge accepts.
"""

import sys
import socket
import base64
import os

ROBOT_IP = "192.168.0.100"
ROBOT_PORT = 8765

key = base64.b64encode(os.urandom(16)).decode('ascii')

variants = [
    # 1. Standard Foxglove with Origin
    ("Foxglove v1 + Origin", b"\r\n".join([
        b"GET / HTTP/1.1",
        f"Host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"Upgrade: websocket",
        b"Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}".encode('ascii'),
        b"Sec-WebSocket-Version: 13",
        b"Sec-WebSocket-Protocol: foxglove.websocket.v1",
        f"Origin: http://{ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"",
        b""
    ])),
    # 2. Foxglove SDK v1
    ("Foxglove SDK v1", b"\r\n".join([
        b"GET / HTTP/1.1",
        f"Host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"Upgrade: websocket",
        b"Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}".encode('ascii'),
        b"Sec-WebSocket-Version: 13",
        b"Sec-WebSocket-Protocol: foxglove.sdk.v1",
        b"",
        b""
    ])),
    # 3. Dual Protocols (v1 + SDK)
    ("Dual (websocket.v1, sdk.v1)", b"\r\n".join([
        b"GET / HTTP/1.1",
        f"Host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"Upgrade: websocket",
        b"Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}".encode('ascii'),
        b"Sec-WebSocket-Version: 13",
        b"Sec-WebSocket-Protocol: foxglove.websocket.v1, foxglove.sdk.v1",
        b"",
        b""
    ])),
    # 4. Canonical Browser Headers (full replica of Chrome)
    ("Full Chrome Replica", b"\r\n".join([
        b"GET / HTTP/1.1",
        f"Host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"Connection: Upgrade",
        b"Pragma: no-cache",
        b"Cache-Control: no-cache",
        b"User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)",
        b"Upgrade: websocket",
        f"Origin: http://localhost:8000".encode('ascii'),
        b"Sec-WebSocket-Version: 13",
        f"Sec-WebSocket-Key: {key}".encode('ascii'),
        b"Sec-WebSocket-Protocol: foxglove.websocket.v1",
        b"",
        b""
    ])),
    # 5. Case-insensitive lowercase headers
    ("All-lowercase headers", b"\r\n".join([
        b"GET / HTTP/1.1",
        f"host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
        b"upgrade: websocket",
        b"connection: upgrade",
        f"sec-websocket-key: {key}".encode('ascii'),
        b"sec-websocket-version: 13",
        b"sec-websocket-protocol: foxglove.websocket.v1",
        b"",
        b""
    ]))
]

print("=" * 70)
print(f"  🔍 Testing 5 Handshake Variations on {ROBOT_IP}:{ROBOT_PORT}")
print("=" * 70)

for name, payload in variants:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((ROBOT_IP, ROBOT_PORT))
        s.sendall(payload)
        resp = s.recv(1024).decode('utf-8', errors='ignore')
        s.close()
        
        status_line = resp.split('\r\n')[0] if '\r\n' in resp else resp[:40]
        if "101" in resp:
            print(f"\n  🎯 [SUCCESS] Variant '{name}' ACCEPTED!")
            print(f"     Response: {status_line}")
        else:
            print(f"  ❌ Variant '{name}': {status_line.strip()} ({resp.strip().replace(chr(10), ' ')[:60]})")
    except Exception as e:
        print(f"  ❌ Variant '{name}': Error {e}")

print("=" * 70)
