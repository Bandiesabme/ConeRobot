#!/usr/bin/env python3
"""
Inspect all advertised Foxglove Bridge channels and message formats on Robot 1
"""

import socket
import json
import time
import base64
import os
import struct

ROBOT_IP = "192.168.0.100"
ROBOT_PORT = 8765

print(f"Connecting to {ROBOT_IP}:{ROBOT_PORT} to inspect published ROS 2 topics...")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(4.0)
s.connect((ROBOT_IP, ROBOT_PORT))

key = base64.b64encode(os.urandom(16)).decode('ascii')
req_lines = [
    b"GET / HTTP/1.1",
    f"Host: {ROBOT_IP}:{ROBOT_PORT}".encode('ascii'),
    b"Upgrade: websocket",
    b"Connection: Upgrade",
    f"Sec-WebSocket-Key: {key}".encode('ascii'),
    b"Sec-WebSocket-Version: 13",
    b"Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1",
    b"",
    b""
]
s.sendall(b"\r\n".join(req_lines))

resp = s.recv(2048).decode('utf-8', errors='ignore')
if "101" not in resp:
    print("Handshake failed:", resp)
    sys.exit(1)

print("✅ Connected! Sending clientInfo...")

# Send clientInfo
client_info = json.dumps({"op": "clientInfo", "name": "TopicInspector"}).encode('utf-8')
frame = bytearray([0x81, 0x80 | len(client_info)])
mask = os.urandom(4)
frame.extend(mask)
frame.extend(b ^ mask[i % 4] for i, b in enumerate(client_info))
s.sendall(frame)

# Helper to read websocket frame
def read_frame():
    head = s.recv(2)
    if len(head) < 2: return None, None
    b1, b2 = head[0], head[1]
    op = b1 & 0x0F
    has_mask = bool(b2 & 0x80)
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack("!H", s.recv(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", s.recv(8))[0]
    mask = s.recv(4) if has_mask else None
    payload = bytearray()
    while len(payload) < length:
        chunk = s.recv(length - len(payload))
        if not chunk: break
        payload.extend(chunk)
    if has_mask:
        payload = bytearray(b ^ mask[i % 4] for i, b in enumerate(payload))
    return op, payload

start_t = time.time()
print("\nWaiting for topic advertisements from robot...\n" + "=" * 70)

while time.time() - start_t < 4.0:
    try:
        op, payload = read_frame()
        if payload is None: break
        if op == 1: # Text (JSON)
            text = payload.decode('utf-8', errors='ignore')
            data = json.loads(text)
            print(f"OP: {data.get('op')}")
            if data.get("op") == "advertise":
                channels = data.get("channels", [])
                print(f"📦 Total Advertised Topics: {len(channels)}\n")
                for ch in channels:
                    print(f"  • Topic: {ch.get('topic'):<30} Schema: {ch.get('schemaName'):<35} Encoding: {ch.get('encoding')}")
                break
            elif data.get("op") == "serverInfo":
                print(f"ℹ️ Server Info: {data}")
    except Exception as e:
        print("Error:", e)
        break

s.close()
print("=" * 70)
