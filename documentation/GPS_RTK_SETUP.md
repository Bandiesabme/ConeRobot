# 📡 Waveshare LC29H(DA) Dual-Band GPS/RTK Setup & Native ROS 2 Guide

This guide details the complete hardware configuration, Raspberry Pi 5 serial port setup (`/dev/ttyAMA0`), and the native **ROS 2 LC29H GPS/RTK Driver & NTRIP Rover Node** (`lc29h_gps_node`) for the **Waveshare LC29H(DA) Dual-Band GPS/RTK HAT (25279)** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

The native driver replaces external demo scripts and supports connecting to both **public NTRIP correction casters** (e.g., RTK2Go) and **your own local/private base stations**.

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
> **Antenna**: Connect the external active multi-band GNSS antenna to the SMA connector and position it outdoors with an unobstructed line-of-sight to the open sky.

---

## 2. Raspberry Pi 5 Serial Port Setup (`/dev/ttyAMA0`)

On the Raspberry Pi 5, the primary hardware UART on the 40-pin GPIO header is named **`/dev/ttyAMA0`** (baud rate: **115200**).

### Step 1: Enable Hardware UART in Boot Config

Add the UART parameters to `/boot/firmware/config.txt`:

```bash
sudo bash -c 'cat << EOF >> /boot/firmware/config.txt
enable_uart=1
dtparam=uart0=on
EOF'
```

### Step 2: Configure Permissions & Mask Conflicting Services

```bash
# 1. Add user to dialout group for serial port access (/dev/ttyAMA0)
sudo usermod -aG dialout $USER

# 2. Disable and permanently MASK the Linux serial login console on ttyAMA0
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service

# 3. Disable gpsd daemon if previously installed (gpsd locks the port)
sudo systemctl stop gpsd gpsd.socket 2>/dev/null || true
sudo systemctl disable gpsd gpsd.socket 2>/dev/null || true
```

Reboot to apply:
```bash
sudo reboot
```

### Step 3: Quick Hardware UART Verification

To verify that the GPS module is actively outputting raw NMEA satellite sentences over hardware UART:

```bash
sudo stty -F /dev/ttyAMA0 115200 raw -echo
sudo cat /dev/ttyAMA0
```
*(You should see live `$GNGGA...` and `$GNRMC...` sentences scrolling. Press `Ctrl+C` to stop).*

---

## 3. Base Station & NTRIP Configuration

The native node `lc29h_gps_node` runs an integrated, auto-reconnecting NTRIP Rover client that receives RTCM3 differential correction packets and writes them directly into the LC29H serial stream.

You can configure your base station settings in `src/cone_robot_control/config/robot_config.yaml`:

```yaml
lc29h_gps_node:
  ros__parameters:
    serial_port: "/dev/ttyAMA0"
    baud_rate: 115200
    frame_id: "gps_link"
    publish_rate_hz: 10.0
    mock_hardware: false

    # --- NTRIP RTK Correction Stream Configuration ---
    ntrip_enable: true
    ntrip_caster: "rtk2go.com"          # Public caster OR your local base station IP (e.g. "192.168.1.50" or "10.42.0.1")
    ntrip_port: 2101                   # Standard NTRIP port
    ntrip_mountpoint: "PFORZEM"         # Caster mountpoint name (e.g. "PFORZEM" or your own custom mountpoint)
    ntrip_user: "conerobot@rover.local" # Account / email
    ntrip_password: "none"             # Password (or "none" for public casters)
    ntrip_send_gga: true               # Send NMEA position feedback to caster for VRS / keepalive
```

### Option A: Using Public Casters (e.g., RTK2Go, CORS, SAPOS)
- Set `ntrip_caster: "rtk2go.com"`
- Set `ntrip_mountpoint: "<NEARBY_MOUNTPOINT>"` (find nearest mountpoint at [rtk2go.com](http://rtk2go.com))
- Set `ntrip_user: "your_email@example.com"`

### Option B: Using Your Own Base Station (e.g., Local ESP32, Pi Base, or Local Caster)
- Set `ntrip_caster: "192.168.1.50"` (or your base station's local IP on the network)
- Set `ntrip_port: 2101`
- Set `ntrip_mountpoint: "MY_BASE"`
- Set `ntrip_user` and `ntrip_password` as configured on your base station.

### Option C: Standalone GNSS Mode (No RTK)
- Set `ntrip_enable: false`. The node will operate as a standard multi-constellation 3D GPS receiver.

---

## 4. How to Launch & Verify with ROS 2

### Step 1: Build Workspace

```bash
cd ~/github/ConeRobot  # or your workspace directory
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch Robot Stack with GPS

```bash
# Launch Motors + IMU + GPS/RTK
ros2 launch cone_robot_control robot.launch.py launch_gps:=true
```

Or run the GPS node individually:
```bash
ros2 run cone_robot_control lc29h_gps_node --ros-args --params-file src/cone_robot_control/config/robot_config.yaml
```

---

## 5. Live Topics & Verification

### A. Monitor Human-Readable RTK Status & Diagnostics
```bash
ros2 topic echo /gps/status
```
*Sample Output:*
```text
data: "Fix: RTK FLOAT | Sats: 35 | HDOP: 0.43 | NTRIP: Connected (142.3 KB RTCM)"
```

### B. Monitor Standard NavSatFix Topic
```bash
ros2 topic echo /fix
```
*Sample Output:*
```yaml
header:
  stamp:
    sec: 1723896500
    nanosec: 123456789
  frame_id: gps_link
status:
  status: 2    # STATUS_GBAS_FIX (Centimeter RTK Positioning)
  service: 15  # GPS + GLONASS + GALILEO + BEIDOU
latitude: 49.0054911
longitude: 8.2457705
altitude: 134.792
position_covariance: [0.0074, 0.0, 0.0, 0.0, 0.0074, 0.0, 0.0, 0.0, 0.0296]
position_covariance_type: 1
```

---

## 6. Troubleshooting & Solutions

| Issue | Root Cause | Solution |
| :--- | :--- | :--- |
| `Permission denied: '/dev/ttyAMA0'` | Serial port owned by `root:dialout` | Run `sudo usermod -aG dialout $USER` and log out/in. |
| `serial-getty` interference / garbled bytes | Linux login console running on UART | Run `sudo systemctl mask serial-getty@ttyAMA0.service` and reboot. |
| `NTRIP connection lost: Reconnecting in 2.0s` | Network dropout or invalid mountpoint | Verify Wi-Fi internet connection and check `ntrip_caster` / `ntrip_mountpoint` in `robot_config.yaml`. |
| Fix stays `3D FIX (SPS)` instead of `RTK` | No RTCM3 base corrections reaching module | Check `/gps/status` to ensure NTRIP is `Connected` and RTCM KB is increasing. |
| Zero satellites or `NO FIX` | Blocked sky view | Move antenna outside with open sky visibility. |
