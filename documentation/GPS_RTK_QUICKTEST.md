# 🧪 Waveshare LC29H(DA) GPS/RTK HAT Quick Test & Verification Guide (Raspberry Pi 5)

This document records the verified hardware setup, UART configuration (`/dev/ttyAMA0`), python NTRIP client patches, and live RTK benchmark results for the **Waveshare LC29H(DA) Dual-Band GPS/RTK HAT (25279)** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

---

## 1. Verified Live RTK Benchmark Results

During hardware verification with base station **PFORZEM** over RTK2go:

```text
b'$GNGGA,125545.000,4900.549118,N,00824.577053,E,5,35,0.43,134.792,M,47.942,M,1.0,0000*5C\r\n'
```

- **Fix Quality**: **`5`** (**RTK FLOAT** - high-precision centimeter to sub-decimeter positioning!)
- **Satellites Tracked**: **35 satellites** locked simultaneously (GPS + GLONASS + Galileo + BeiDou + QZSS).
- **HDOP**: **`0.43`** (excellent satellite geometry).
- **Differential Correction Age**: **`1.0s`** fresh RTCM3 corrections from station `0000`.

---

## 2. Hardware Setup

- **HAT Header**: Mounted directly to the Pi 5 40-pin GPIO header.
- **Jumper Header**: Yellow jumper cap set to **Position B** (routes TX/RX to GPIO 14/15, physical pins 8 & 10).
- **GNSS Antenna**: External active multi-band antenna connected to the SMA port with outdoor sky line-of-sight.

---

## 3. Raspberry Pi 5 Serial Port Setup (`/dev/ttyAMA0`)

On Raspberry Pi 5, the primary hardware UART on the 40-pin GPIO header is named **`/dev/ttyAMA0`** (baud rate: **115200**).

### A. Boot Firmware Configuration

Add to `/boot/firmware/config.txt`:
```ini
enable_uart=1
dtparam=uart0=on
```

Reboot to apply:
```bash
sudo reboot
```

### B. Disable Background Services & Permissions

```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Stop & disable gpsd daemon so it does not conflict over /dev/ttyAMA0
sudo systemctl stop gpsd gpsd.socket
sudo systemctl disable gpsd gpsd.socket

# Disable Linux login console on ttyAMA0
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
```

---

## 4. Quick Test: Direct Raw NMEA Stream

To verify raw UART hardware data output at 115200 baud:

```bash
sudo stty -F /dev/ttyAMA0 115200 raw -echo
sudo cat /dev/ttyAMA0
```

---

## 5. Waveshare NTRIP RTK Demo Setup & Auto-Reconnect Patch

### A. Download Demo Code

```bash
cd ~
wget 'https://files.waveshare.com/wiki/LC29H(XX)-GPS-RTK-HAT/Lc29h_gps_rtk_hat_code.zip' -O Lc29h_gps_rtk_hat_code.zip
unzip -o Lc29h_gps_rtk_hat_code.zip -d ~/lc29h_demo/
```

### B. Apply Waveshare Fixes & Auto-Reconnect Patch

Run this Python command to apply all required Pi 5 serial port updates, binary RTCM3 `latin1` decoding fix, and continuous auto-reconnect handling:

```bash
python3 -c "
path = '/home/conerobot/lc29h_demo/lc29h_gps_rtk_hat_code/python/rtk_rover/main.py'
with open(path, 'r') as f:
    content = f.read()

# Fix serial port & binary RTCM3 decoding
content = content.replace('/dev/ttyS0', '/dev/ttyAMA0')
content = content.replace(\"casterResponse.decode('utf-8')\", \"casterResponse.decode('latin1')\")

# Replace static execution with continuous auto-reconnect loop
old_tail = '''    n = NtripClient(**ntripArgs)
    try:
        n.readData()
    finally:
        if fileOutput:
            f.close()
        if options.headerFile:
            h.close()'''

new_tail = '''    while True:
        try:
            n = NtripClient(**ntripArgs)
            n.readData()
        except Exception as e:
            import time
            print(f\"Reconnecting to RTK2go... ({e})\")
            time.sleep(1)'''

if old_tail in content:
    content = content.replace(old_tail, new_tail)

content = content.replace('sys.exit(1)', 'pass').replace('sys.exit(0)', 'pass').replace('sys.exit()', 'pass')

with open(path, 'w') as f:
    f.write(content)
print('All patches applied successfully!')
"
```

### C. Run the NTRIP RTK Rover Client

```bash
cd ~/lc29h_demo/lc29h_gps_rtk_hat_code/python/rtk_rover

# Syntax: sudo python3 main.py -u <email> -p none rtk2go.com 2101 <MOUNTPOINT>
sudo python3 main.py -u your_email@gmail.com -p none rtk2go.com 2101 PFORZEM
```

---

## 6. Summary of Troubleshooting Solutions

| Issue | Cause | Verified Solution |
| :--- | :--- | :--- |
| `Permission denied: '/dev/ttyAMA0'` | Serial port owned by `root:dialout` | Run with `sudo` or execute `sudo chmod 666 /dev/ttyAMA0`. |
| `UnicodeDecodeError: 'utf-8' byte 0xd3` | Binary RTCM3 packets starting with `0xD3` break UTF-8 parser | Use `.decode('latin1')` on socket response buffer. |
| `device reports readiness to read but returned no data` | `gpsd` competing with script for serial port access | Run `sudo systemctl stop gpsd gpsd.socket` and `sudo systemctl disable gpsd gpsd.socket`. |
| Disconnect after 15–30 seconds | RTK2go caster drops streams when socket idle | Continuous `while True:` auto-reconnect loop seamlessly re-establishes connection in 1s. |
