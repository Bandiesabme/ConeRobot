#!/usr/bin/env python3
"""
Inspect and stream /foxglove_bridge/sysinfo directly in terminal
"""

import socket
import json
import time
import base64
import os
import struct

ROBOT_IP = "192.168.0.100"
ROBOT_PORT = 8765

print(f"Connecting to {ROBOT_IP}:{ROBOT_PORT} to listen for /foxglove_bridge/sysinfo...")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5.0)
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
    exit(1)

print("✅ Handshake 101 OK! Sending clientInfo...")

# Send clientInfo
client_info = json.dumps({"op": "clientInfo", "name": "SysInfoListener"}).encode('utf-8')
frame = bytearray([0x81, 0x80 | len(client_info)])
mask = os.urandom(4)
frame.extend(mask)
frame.extend(b ^ mask[i % 4] for i, b in enumerate(client_info))
s.sendall(frame)

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

sysinfo_channel_id = None
sub_id = 100

start_t = time.time()
print("\nWaiting for topic list & subscribing...")

while time.time() - start_t < 10.0:
    try:
        op, payload = read_frame()
        if payload is None: break
        
        # Text Frame (JSON)
        if op == 1:
            text = payload.decode('utf-8', errors='ignore')
            data = json.loads(text)
            if data.get("op") == "advertise":
                for ch in data.get("channels", []):
                    if ch.get("topic") == "/foxglove_bridge/sysinfo":
                        sysinfo_channel_id = ch.get("id")
                        print(f"🎯 Found /foxglove_bridge/sysinfo (Channel ID: {sysinfo_channel_id}, Encoding: {ch.get('encoding')})")
                        sub_msg = json.dumps({"op": "subscribe", "subscriptions": [{"id": sub_id, "channelId": sysinfo_channel_id}]}).encode('utf-8')
                        sub_frame = bytearray([0x81, 0x80 | len(sub_msg)])
                        mask = os.urandom(4)
                        sub_frame.extend(mask)
                        sub_frame.extend(b ^ mask[i % 4] for i, b in enumerate(sub_msg))
                        s.sendall(sub_frame)
                        print("📡 Sent subscription request for sysinfo! Listening for packets...")
            elif data.get("op") == "message":
                print(f"\n[JSON MESSAGE RECEIVED]:\n{data.get('data')}")

        # Binary Frame
        elif op == 2:
            if len(payload) >= 13:
                raw_sub_id = struct.unpack("<I", payload[1:5])[0]
                if raw_sub_id == sub_id:
                    raw_text = payload[13:].decode('utf-8', errors='ignore')
                    print(f"\n[BINARY SYSINFO RECEIVED]:\n{raw_text}")
    except Exception as e:
        print("Error:", e)
        break

s.close()
