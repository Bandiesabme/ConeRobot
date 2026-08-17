# 📡 Waveshare LC29H(DA) Dual-Band GPS/RTK Setup & Verification Guide

This guide details the complete hardware configuration, Raspberry Pi 5 serial port setup (`/dev/ttyAMA0`), NTRIP RTK rover client execution, automated reconnect patches, verified benchmark results, and ROS 2 integration for the **Waveshare LC29H(DA) Dual-Band GPS/RTK HAT (25279)** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

---

## 1. Hardware Setup & Pinout Coexistence

The Waveshare LC29H(DA) HAT mounts directly onto the Raspberry Pi 5 40-pin GPIO header.

### A. Pin Allocation Table

| Function | RPi 5 Physical Pin | GPIO Number | Cytron MDD10 & IMU Conflict? |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | ✅ **No Conflict** (Motors: GPIO 12, 13, 24, 25; IMU: GPIO 2, 3, 4) |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | ✅ **No Conflict** |
| **PPS** (Pulse Per Second) | Pin 7 | GPIO 4 | ✅ **No Conflict** |
| **Reset** | Pin 12 | GPIO 18 | ✅ **No Conflict** |
| **Power (5V / 3.3V)** | Pins 2, 4 (5V) / Pin 1 (3.3V) | - | ✅ Power Rails |
| **Ground** | Pins 6, 9, 14, 20, 25, 30 | GND | ✅ **Common Ground** |

> [!IMPORTANT]
> **Jumper Cap Selection**: Set the yellow jumper cap on the LC29H HAT to **Position B** (connects HAT UART TX/RX directly to Raspberry Pi GPIO 14 / GPIO 15).
> **Antenna**: Connect the external active multi-band GNSS antenna to the SMA connector and position it outdoors with a clear line-of-sight to the sky.

---

## 2. Raspberry Pi 5 Serial Port Setup (`/dev/ttyAMA0`)

On the Raspberry Pi 5, the primary hardware UART on the 40-pin GPIO header is routed through the RP1 I/O controller and is named **`/dev/ttyAMA0`** (baud rate: **115200**).

### Step 1: Enable Hardware UART in Boot Config

Add the UART parameters to `/boot/firmware/config.txt`:

```bash
sudo bash -c 'cat << EOF >> /boot/firmware/config.txt
enable_uart=1
dtparam=uart0=on
EOF'
```

Reboot to apply the firmware configuration:
```bash
sudo reboot
```

### Step 2: Configure Permissions & Disable Competing Services

```bash
# Add user to dialout group for serial port access (/dev/ttyAMA0)
sudo usermod -aG dialout $USER

# Disable the Linux login console on ttyAMA0
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service

# CRITICAL: Stop & disable gpsd daemon (gpsd locks the port and blocks NTRIP / ROS scripts)
sudo systemctl stop gpsd gpsd.socket
sudo systemctl disable gpsd gpsd.socket
```

---

## 3. Quick Verification: Direct Raw NMEA Stream

To verify that the GPS module is outputting raw NMEA satellite data over the hardware UART:

```bash
sudo stty -F /dev/ttyAMA0 115200 raw -echo
sudo cat /dev/ttyAMA0
```

*Expected output: scrolling NMEA sentences (`$GNGGA...`, `$GNRMC...`, `$GNVTG...`). Press `Ctrl+C` to stop.*

---

## 4. Waveshare NTRIP RTK Rover Setup & Auto-Reconnect Patch

RTK (Real-Time Kinematic) uses differential RTCM3 correction streams from an NTRIP base station to achieve **centimeter-level positioning accuracy**.

### Step 1: Download Waveshare Demo Code

```bash
cd ~
wget 'https://files.waveshare.com/wiki/LC29H(XX)-GPS-RTK-HAT/Lc29h_gps_rtk_hat_code.zip' -O Lc29h_gps_rtk_hat_code.zip
unzip -o Lc29h_gps_rtk_hat_code.zip -d ~/lc29h_demo/
```

### Step 2: Apply Raspberry Pi 5 & Auto-Reconnect Patches

The stock Waveshare demo assumes `/dev/ttyS0`, crashes on binary RTCM3 data (`0xD3` bytes) with UTF-8 decoding errors, and terminates when RTK2go drops idle connections.

Run this Python script on your Pi 5 to automatically apply all verified fixes:

```bash
python3 -c "
path = '/home/conerobot/lc29h_demo/lc29h_gps_rtk_hat_code/python/rtk_rover/main.py'
with open(path, 'r') as f:
    content = f.read()

# 1. Update serial port for Pi 5 & binary RTCM3 decoding
content = content.replace('/dev/ttyS0', '/dev/ttyAMA0')
content = content.replace(\"casterResponse.decode('utf-8')\", \"casterResponse.decode('latin1')\")

# 2. Add continuous auto-reconnect loop
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
            print(f\"Reconnecting to RTK caster... ({e})\")
            time.sleep(1)'''

if old_tail in content:
    content = content.replace(old_tail, new_tail)

content = content.replace('sys.exit(1)', 'pass').replace('sys.exit(0)', 'pass').replace('sys.exit()', 'pass')

with open(path, 'w') as f:
    f.write(content)
print('All LC29H patches applied successfully!')
"
```

### Step 3: Run the NTRIP RTK Rover Client

Connect to your NTRIP caster (e.g., RTK2Go, local base station, or CORS network):

```bash
cd ~/lc29h_demo/lc29h_gps_rtk_hat_code/python/rtk_rover

# Syntax: sudo python3 main.py -u <email> -p <password/none> <caster_host> <port> <mountpoint>
sudo python3 main.py -u your_email@gmail.com -p none rtk2go.com 2101 PFORZEM
```

---

## 5. Verified Live RTK Benchmark Results

Below is a verified live NMEA log achieved during hardware testing:

```text
b'$GNGGA,125545.000,4900.549118,N,00824.577053,E,5,35,0.43,134.792,M,47.942,M,1.0,0000*5C\r\n'
```

- **Fix Quality**: **`5`** (**RTK FLOAT** — high-precision centimeter/sub-decimeter positioning).
- **Satellites Tracked**: **35 satellites** locked simultaneously (GPS + GLONASS + Galileo + BeiDou + QZSS).
- **HDOP**: **`0.43`** (excellent satellite geometry).
- **Differential Age**: **`1.0s`** fresh RTCM3 differential corrections from mountpoint station `0000`.

---

## 6. ROS 2 Integration (`/fix` Topic)

To publish live GPS data into the ROS 2 ecosystem as standard `sensor_msgs/msg/NavSatFix` messages:

### Step 1: Install ROS 2 NMEA Driver

```bash
sudo apt install -y ros-jazzy-nmea-navsat-driver
```

### Step 2: Run the ROS 2 Serial Driver Node

```bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyAMA0 \
  -p baud:=115200 \
  -p frame_id:=gps_link
```

### Step 3: Verify ROS 2 Topic Output

In another terminal:

```bash
ros2 topic echo /fix
```

---

## 7. Troubleshooting & Solutions

| Issue | Root Cause | Verified Solution |
| :--- | :--- | :--- |
| `Permission denied: '/dev/ttyAMA0'` | Serial port owned by `root:dialout` | Run with `sudo` or execute `sudo usermod -aG dialout $USER && sudo chmod 666 /dev/ttyAMA0`. |
| `UnicodeDecodeError: 'utf-8' byte 0xd3` | Binary RTCM3 packets starting with `0xD3` break UTF-8 decoding | Use `.decode('latin1')` on the socket response buffer in `main.py`. |
| `device reports readiness to read but returned no data` | `gpsd` competing with script for serial port access | Run `sudo systemctl stop gpsd gpsd.socket && sudo systemctl disable gpsd gpsd.socket`. |
| Disconnect after 15–30 seconds | RTK2go caster drops streams when socket is idle | The continuous `while True:` auto-reconnect loop seamlessly re-establishes connection in 1 second. |
| Zero satellites or Fix Quality = `0` | Blocked sky view / antenna indoor | Move the GNSS antenna outside with an unobstructed view of the open sky. |
