# 📡 Raspberry Pi 5 RTK Base Station Setup & Caster Guide

This guide details how to configure a second **Raspberry Pi 5** equipped with an RTK Base GNSS module (such as the **Waveshare LC29H(BS)** or **LC29H(EA)**) as a dedicated **Local RTK Base Station** broadcasting live centimeter-accuracy RTCM3 correction streams to your Cone Robot over Wi-Fi.

---

## 1. Hardware Pinout & Header Setup

Mount your RTK Base GNSS HAT directly to the Raspberry Pi 5 40-pin GPIO header:

| Pin Function | RPi 5 Physical Pin | GPIO Number | Notes |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | Routes through RP1 controller |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | Routes through RP1 controller |
| **Power (5V / 3.3V)** | Pins 2, 4 (5V) / Pin 1 (3.3V) | - | Clean power rail |
| **Ground** | Pins 6, 9, 14, 20, 25, 30 | GND | Common Ground |

> [!IMPORTANT]
> - **Yellow Jumper Cap**: Set to **Position B** (connects HAT UART to GPIO 14/15 -> `/dev/ttyAMA0`).
> - **Base Antenna Placement**: Mount the external GNSS antenna outdoors with an unobstructed 360° view of the sky on top of a metallic ground plane (e.g., 10–15 cm metal plate/disc) for maximum satellite signal purity.

---

## 2. Base Pi Initial System Setup

Run these commands once on your new **Base Station Pi**:

```bash
# 1. Enable hardware UART in firmware boot config
sudo bash -c 'cat << EOF >> /boot/firmware/config.txt
enable_uart=1
dtparam=uart0=on
EOF'

# 2. Add user to dialout group for UART access
sudo usermod -aG dialout $USER

# 3. Disable & mask Linux serial login console
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service

# 4. Install Python serial dependencies
sudo apt update
sudo apt install -y python3-pip python3-serial git

# 5. Clone repository on Base Pi
cd ~
git clone https://github.com/Bandiesabme/ConeRobot.git ~/github/ConeRobot

# 6. Apply Wi-Fi auto-connect to router
sudo bash ~/github/ConeRobot/documentation/scripts/setup_wifi.sh

# Reboot to apply all firmware & permission changes
sudo reboot
```

---

## 3. Launching the Local Base Station Caster

### Option A: Manual Terminal Launch

In your Base Pi terminal:
```bash
python3 ~/github/ConeRobot/documentation/scripts/base_station_caster.py --port 2101 --mountpoint BASE
```

*Expected output:*
```text
=================================================================
  📡 RASPBERRY PI 5 RTK BASE STATION NTRIP CASTER
=================================================================
  • Serial Port       : /dev/ttyAMA0 @ 115200 baud
  • NTRIP Server Port : 2101
  • Mountpoint        : /BASE
  • Connection URL    : http://192.168.0.20:2101/BASE
=================================================================

[NTRIP Server] Listening for rovers on port 2101...
[Serial] Opening Base GNSS UART: /dev/ttyAMA0 @ 115200 baud...
✅ [Serial] Base GNSS UART active! Streaming RTCM3 packets...
```

---

### Option B: Auto-Start on Boot (Systemd Background Service)

To make the Base Station automatically start streaming RTCM3 the moment power is connected in the field (no SSH or keyboard needed):

```bash
sudo tee /etc/systemd/system/ntrip-base.service << 'EOF'
[Unit]
Description=RTK Base Station NTRIP Caster
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=conerobot
WorkingDirectory=/home/conerobot/github/ConeRobot
ExecStart=/usr/bin/python3 /home/conerobot/github/ConeRobot/documentation/scripts/base_station_caster.py --port 2101 --mountpoint BASE
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable --now ntrip-base.service

# Check service status
sudo systemctl status ntrip-base.service
```

---

## 4. Connecting Your Cone Robot Rover to the Base Station

On your **Robot Pi**, update `src/cone_robot_control/config/robot_config.yaml`:

```yaml
lc29h_gps_node:
  ros__parameters:
    serial_port: "/dev/ttyAMA0"
    baud_rate: 115200
    frame_id: "gps_link"
    
    # --- Connect to Your Local Base Station ---
    ntrip_enable: true
    ntrip_caster: "192.168.0.20"       # Replace with your Base Pi's actual IP address
    ntrip_port: 2101
    ntrip_mountpoint: "BASE"           # Matches the base station mountpoint
    ntrip_user: "conerobot"
    ntrip_password: "none"
```

Then launch your robot:
```bash
ros2 launch cone_robot_control robot.launch.py
```

### Result:
Your robot will connect to your local base station over the Wi-Fi router and achieve **instant `RTK FIX` (< 1 cm error)** because the baseline distance is less than 50 meters!
