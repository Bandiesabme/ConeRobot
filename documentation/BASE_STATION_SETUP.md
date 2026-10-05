# 📡 Raspberry Pi Zero RTK Base Station Setup & Live Web Dashboard Guide

This guide details how to configure a dedicated **Raspberry Pi Zero (W / Zero 2 W)** equipped with an RTK Base GNSS module (such as the **Waveshare LC29H(BS)** or **LC29H(EA)**) as an ultra-low-power, standalone **Field RTK Base Station**.

The Pi Zero hosts both:
1. **Local NTRIP Caster** (port `2101`) to broadcast live centimeter-accuracy RTCM3 differential corrections to your Cone Robot over Wi-Fi.
2. **Real-Time Web Dashboard** (port `8080`) accessible on any phone, tablet, or laptop browser to monitor Survey-In progress, accuracy standard deviation, satellite lock, and connected rovers.



## 1. Hardware Pinout & Header Setup

Mount your RTK Base GNSS HAT directly to the Raspberry Pi Zero 40-pin GPIO header:

| Pin Function | Pi Zero Physical Pin | GPIO Number | Notes |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | High-speed PL011 UART transmit |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | High-speed PL011 UART receive |
| **Power (5V)** | Pin 2 or 4 | 5V | Powers LC29H module (~50 mA) |
| **Ground** | Pin 6, 9, 14, 20, 25, 30 | GND | Common Ground |

> [!IMPORTANT]
> - **Yellow Jumper Cap on HAT**: Set to **Position B** (routes GNSS UART directly to 40-pin GPIO header pins 14/15 -> `/dev/ttyAMA0`).
> - **Base Antenna Placement**: Mount the external multi-band GNSS antenna outdoors with an unobstructed 360° view of the open sky. Place it on top of a metallic ground plane (e.g., a 10–15 cm metal disc or tin lid) to reject ground-bounce multipath signals and maximize carrier-to-noise ratio ($C/N_0$).

---

## 2. System Prerequisites

Before launching the base station software, ensure your Raspberry Pi Zero has been prepared following the dedicated **[Raspberry Pi Zero Setup Guide](RPI_ZERO_SETUP.md)**:
- [x] Base packages installed (`git`, `python3`, `python3-serial`).
- [x] Hardware PL011 UART enabled (`/dev/ttyAMA0` configured via `dtoverlay=disable-bt`).
- [x] Serial login console disabled and user added to `dialout` group.
- [x] Wi-Fi configured and power saving disabled (`wifi.powersave = 2`).
- [x] Repository cloned to `~/github/ConeRobot`.

---

## 3. Testing the Base Station Caster & Web Dashboard

### Manual Test Run
Run the caster script directly in your terminal:

```bash
python3 ~/github/ConeRobot/scripts/base_station_caster.py --port 2101 --web-port 8080 --mountpoint BASE
```

*Expected Terminal Output:*
```text
===========================================================================
  📡 RASPBERRY PI RTK BASE STATION & WEB DASHBOARD
===========================================================================
  • Base Station IP   : 192.168.137.105 (or 192.168.0.x)
  • Serial Port       : /dev/ttyAMA0 @ 115200 baud
  • NTRIP Server Port : 2101 (Mountpoint: /BASE)
  • 🌐 Web Dashboard  : http://192.168.137.105:8080
  • Mode              : ⏳ Auto-Calibrating (3600s Target)
===========================================================================

[NTRIP Server] Listening for rovers on port 2101...
[Serial] Opening Serial Port: /dev/ttyAMA0 @ 115200 baud...
✅ [Serial] Base GNSS UART active! Monitoring Survey-In & streaming RTCM3...
```

Press `Ctrl+C` to stop the test once verified.

---

## 4. Live Browser Web Dashboard

Open any web browser on your phone, tablet, or laptop connected to the same Wi-Fi network:
```text
http://<BASE_PI_ZERO_IP>:8080
```
*(Example: `http://192.168.137.105:8080` or `http://conerobotBaseStation.local:8080`)*

### Dashboard Features:
- 🎯 **Survey-In Progress Bar**: Real-time calibration countdown (`0%` $\rightarrow$ `100%`), elapsed duration, and standard deviation accuracy (`0.35 m`).
- 🛰️ **Tracked Satellite Constellations**: Real-time count of GPS, GLONASS, Galileo, and BeiDou satellites.
- 📍 **Fixed Reference Coordinates**: Absolute Latitude, Longitude, and Ellipsoidal Height with a direct **Google Maps** link.
- 🔒 **"Lock Now" & Auto-Restore**: Locks current averaged position or automatically restores previous survey coordinates on boot (0 mm drift).
- 📍 **Saved Location Profiles (Presets)**: Save multiple physical benchmark spots (e.g. "Home Yard", "University Roof", "Spot A"). Switch between them with 1 click from the dropdown without recalibrating!
- 📡 **Active Connected Rovers**: Live table of all robots receiving RTCM3 corrections and byte throughput.
- 📋 **Rover Config Snippet**: Instant copyable YAML parameters formatted for `robot_config.yaml`.

---

## 5. Auto-Start on Boot (Systemd Background Service)

Configure the Base Station to automatically start on boot so it runs headless in the field:

```bash
  # Create systemd service unit
  sudo tee /etc/systemd/system/ntrip-base.service << 'EOF'
  [Unit]
  Description=RTK Base Station NTRIP Caster & Web Dashboard
  After=network.target
  Wants=network.target

  [Service]
  Type=simple
  User=conerobot
  WorkingDirectory=/home/conerobot/github/ConeRobot
  ExecStart=/usr/bin/python3 /home/conerobot/github/ConeRobot/scripts/base_station_caster.py --port 2101 --web-port 8080 --mountpoint BASE
  Restart=always
  RestartSec=3

  [Install]
  WantedBy=multi-user.target
  EOF

  # Reload systemd, enable service on boot, and start it immediately
  sudo systemctl daemon-reload
  sudo systemctl enable --now ntrip-base.service

  # Verify service is active and running
  sudo systemctl status ntrip-base.service
  ```

  To view live background logs at any time:
  ```bash
  journalctl -u ntrip-base.service -f
```



## 6. Connecting the Cone Robot (Rover) to the Base Station

On your **Robot Raspberry Pi 5**, edit `src/cone_robot_control/config/robot_config.yaml`:

```yaml
lc29h_gps_node:
  ros__parameters:
    serial_port: "/dev/ttyAMA0"
    baud_rate: 115200
    frame_id: "gps_link"
    
    # --- Local Base Station NTRIP Configuration ---
    ntrip_enable: true
    ntrip_caster: "conerobotBaseStation.local"  # Auto-resolves Pi Zero on ANY router or hotspot via mDNS
    ntrip_port: 2101
    ntrip_mountpoint: "BASE"
    ntrip_user: "conerobot"
    ntrip_password: "none"
```

Then start the robot software stack:
```bash
ros2 launch cone_robot_control robot.launch.py
```

### Expected Result:
1. `lc29h_gps_node` connects to the Pi Zero NTRIP caster (`192.168.137.105:2101/BASE`).
2. Differential RTCM3 correction frames stream into the robot at 1 Hz.
3. The robot GNSS status transitions from `3D FIX` $\rightarrow$ `RTK FLOAT` $\rightarrow$ **`RTK FIX (14+ Sats, < 1 cm Error)`**!
4. The Base Station web dashboard (`http://<BASE_IP>:8080`) lists your Cone Robot under **Connected Rovers** in real time.

