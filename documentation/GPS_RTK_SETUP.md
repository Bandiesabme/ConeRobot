# 📡 Waveshare LC29H(DA) Dual-Band GPS/RTK HAT Setup Guide

This guide details the complete hardware configuration, Raspberry Pi 5 serial setup (`/dev/ttyAMA0`), NTRIP RTK rover client execution, and ROS 2 integration for the **Waveshare LC29H(DA) Dual-Band GPS/RTK HAT (25279)** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

---

## 1. Hardware Pinout & Coexistence

The Waveshare LC29H(DA) HAT connects directly to the Raspberry Pi 5 40-pin GPIO header.

### A. Pin Allocation Table

| Function | RPi 5 Physical Pin | GPIO Number | Cytron MDD10 Motor Conflict? |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | ✅ **No Conflict** (Motors use GPIO 12, 13, 24, 25) |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | ✅ **No Conflict** |
| **PPS** (Pulse Per Second) | Pin 7 | GPIO 4 | ✅ **No Conflict** |
| **Reset** | Pin 12 | GPIO 18 | ✅ **No Conflict** |
| **Power** | Pin 2, 4 (5V) / Pin 1 (3.3V) | - | ✅ Power Rails |
| **Ground** | Pin 6, 9, 14, 20, 25, 30 | GND | ✅ **Common Ground** |

> [!IMPORTANT]
> **Jumper Cap Selection**: Set the yellow jumper cap on the LC29H HAT to **Position B** (connects HAT UART TX/RX directly to RPi GPIO 14/15).

---

## 2. Raspberry Pi 5 Serial Port Setup (`/dev/ttyAMA0`)

On Raspberry Pi 5, the primary hardware UART on the 40-pin GPIO header is named **`/dev/ttyAMA0`** (unlike Pi 4B which used `/dev/ttyS0`).

### Step 1: Install Dependencies

Run on your Raspberry Pi 5 (`conerobot@conerobot`):

```bash
sudo apt update
sudo apt install -y gpsd gpsd-clients python3-pip unzip
sudo pip3 install gps3 pyserial pynmeagps --break-system-packages

# Add user to dialout group for serial port permission (/dev/ttyAMA0)
sudo usermod -aG dialout $USER
```

### Step 2: Enable Hardware UART in Boot Configuration

Ensure hardware UART is enabled in `/boot/firmware/config.txt`:

```bash
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
```

### Step 3: Disable Linux Serial Terminal Service

Disable the Linux login console on `ttyAMA0` so it does not interfere with GPS NMEA streams:

```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
```

### Step 4: Configure `gpsd` Service

Edit `/etc/default/gpsd`:

```bash
sudo nano /etc/default/gpsd
```

Set the contents to:

```bash
# Configuration for gpsd
START_DAEMON="true"
USBAUTO="false"
DEVICES="/dev/ttyAMA0"
GPSD_OPTIONS="-F /var/run/gpsd.sock"
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`), then restart `gpsd`:

```bash
sudo systemctl restart gpsd
```

---

## 3. Waveshare Demo & NTRIP RTK Rover Setup

RTK (Real-Time Kinematic) uses correction data streams (RTCM3) from a base station over NTRIP to achieve **1–2 centimeter positioning accuracy**.

### Step 1: Download & Extract Waveshare Demo Code

```bash
cd ~
wget 'https://files.waveshare.com/wiki/LC29H(XX)-GPS-RTK-HAT/Lc29h_gps_rtk_hat_code.zip' -O Lc29h_gps_rtk_hat_code.zip
unzip Lc29h_gps_rtk_hat_code.zip -d lc29h_demo
```

### Step 2: Run NTRIP RTK Rover Client

Connect your Pi 5 to your NTRIP Caster server (e.g., RTK2Go, local base station, or official government RTK network):

```bash
cd ~/lc29h_demo/python/rtk_rover/

# Syntax: python3 main.py -u <username> -p <password> <caster_address> <port> <mountpoint>
python3 main.py -u test@example.com -p mypassword rtk2go.com 2101 MY_MOUNTPOINT
```

---

## 4. Diagnostics & Verification

### A. Raw Serial Stream Test

To test direct serial communication at 115200 baud:

```bash
sudo systemctl stop gpsd
sudo stty -F /dev/ttyAMA0 115200
sudo cat /dev/ttyAMA0
```
*(You will see live NMEA sentences starting with `$GNGGA...` and `$GNRMC...` scrolling)*.

### B. Visual Terminal Dashboard (`cgps`)

Start `gpsd` and run `cgps`:

```bash
sudo systemctl restart gpsd
cgps
```

- When satellite lock is established, `cgps` will display live Latitude, Longitude, Altitude, and Fix Status (`3D FIX` or `RTK FIX`).

---

## 5. ROS 2 Integration (`/fix` Topic)

To expose GPS data to ROS 2 Nav2 navigation stacks as `sensor_msgs/msg/NavSatFix`:

```bash
sudo apt install -y ros-jazzy-nmea-navsat-driver
```

Launch the ROS 2 NMEA driver node on Pi 5:

```bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyAMA0 -p baud:=115200 -p frame_id:=gps_link
```

This publishes live GPS position fix data on ROS 2 topic **`/fix`**!
