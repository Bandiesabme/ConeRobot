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

## 1.1 Optional: Battery Voltage & Current Monitor (CJMCU-219 / INA219)

Unlike the robot (which draws 20–50A motor current), the **Raspberry Pi Zero + GNSS Base HAT only draws ~150–300 mA (<0.5 A)**. 
Therefore, you can connect the INA219 **in series** to monitor **Voltage, Current (mA), and Power (W)** simultaneously without any risk to the 3.2A-rated $0.1\,\Omega$ shunt:

| CJMCU-219 Pin | Connection | Function | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **Pin 1** (or 17) | 3.3V Logic | Protects Pi Zero GPIO logic |
| **GND** | **Pin 9** (or 6, 14) | Common Ground | System ground (Battery Negative connects here) |
| **SDA** | **Pin 3** | GPIO 2 (I2C1 SDA) | I2C Data line |
| **SCL** | **Pin 5** | GPIO 3 (I2C1 SCL) | I2C Clock line |
| **VIN+** | **Battery Positive (+)** | Shunt Input | Battery (+) cable feeds into `VIN+` |
| **VIN-** | **Pi Zero / DC-DC 5V (+)** | Shunt Output (Load) | Powers the Pi Zero / buck converter |

```text
[ Battery (+) ] ────────► VIN+ [ INA219 ] VIN- ────────► [ Pi Zero / DC-DC 5V (+) ]
                                   │
[ Battery (–) ] ────────┬──────────┴───────────────────► [ Pi Zero GND (Pin 9) ]
                        └── (COMMON SYSTEM GROUND)
```

> [!NOTE]
> - **Voltage Drop**: At 250 mA, voltage drop across the $0.1\,\Omega$ shunt is only $0.025\text{ V}$ (negligible).
> - **Current & Power**: Measures real-time current draw in mA and total power in Watts.
> - **Safety Rule**: **NEVER** connect `VIN-` directly to `GND`! `VIN-` is the power feed to your Pi Zero / DC-DC converter.

---

## 1.2 Optional: Hardware Pushbutton (GPIO 17 / D17)

A physical tactile button wired to GPIO 17 provides real-time button push status on the Web Dashboard:

| Button Connection | Pi Zero Header | Mode / Function | Notes |
| :--- | :--- | :--- | :--- |
| **Terminal 1** | **Pin 11** (GPIO 17 / D17) | Input with Internal Pull-Up | Pulled HIGH (3.3V) when released |
| **Terminal 2** | **Pin 9** (or Pin 6, 14, 20) | Ground (`GND`) | Connects GPIO 17 to GND when pressed |

- **Terminal Test**: Run `python3 ~/github/ConeRobot/scripts/test_base_button.py`
- **Dashboard Indicator**: Displays `🔘 D17: RELEASED` (gray) vs `🟢 PUSHED` (glowing emerald green) and logs live state in the **Battery, Power & Thermal Health** box.
- **🛑 Safe Shutdown Feature (3s Hold)**:
  - **Press & hold for 3 seconds**: Triggers graceful Linux shutdown (`sudo shutdown -h now`), syncing filesystems and unmounting the SD card safely.
  - **The Safe Power-Down Sequence**:
    1. Hold button for **3 seconds** $\rightarrow$ UI alerts `🛑 SHUTTING DOWN`.
    2. Watch the green Activity (ACT) LED on the Pi Zero: it will blink 10 times in a slow pulse and turn dark.
    3. **Flip your physical toggle switch to OFF**: Cuts power completely so the 5V booster doesn't slowly drain the 18650 cell in storage.
  - **Powering back ON**: Simply flip the toggle switch back to ON — the Pi Zero boots immediately.

---

## 2. Fresh SD Card Setup & Private Repo Deploy Key Guide

If you just flashed a fresh image with **Raspberry Pi Imager** (Raspberry Pi OS Lite), follow these steps to configure packages, set up GitHub deploy keys for the private repository, and prepare the hardware UART:

### Step 2.1: SSH In & Install Base Packages
Connect over SSH (`ssh conerobot@conerobotBaseStation.local` or via IP) and install the required serial and python packages:

```bash
sudo apt update && sudo apt install -y git python3 python3-serial python3-smbus i2c-tools
```

### Step 2.2: Generate GitHub Deploy Key (Private Repo Access)
Because the repository is private, use a dedicated **Read-Only Deploy Key** on the Pi Zero (never use personal passwords or personal tokens that expire):

1. **Generate the key on the Pi Zero**:
   ```bash
   ssh-keygen -t ed25519 -C "conerobot-base-station" -f ~/.ssh/id_ed25519 -N ""
   ```

2. **Display and copy your public key**:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
   *(Select and copy the output string starting with `ssh-ed25519 AAAA...`)*

3. **Add the Deploy Key in GitHub**:
   - Open your repo on GitHub: **Settings** $\rightarrow$ **Deploy keys** (in the left sidebar under *Security*).
   - Click **Add deploy key**.
   - **Title**: `Base Station Pi Zero`
   - **Key**: Paste the `ssh-ed25519 ...` string.
   - **Allow write access**: **Leave UNCHECKED** (Read-only is safest for robots and field stations).
   - Click **Add key**.

### Step 2.3: Clone the Private Repository
Now clone the repo over SSH into `~/github/ConeRobot`:

```bash
mkdir -p ~/github && cd ~/github
git clone git@github.com:Bandiesabme/ConeRobot.git
```
*(When prompted: `Are you sure you want to continue connecting (yes/no/[fingerprint])?`, type `yes` and press Enter).*

### Step 2.4: Hardware PL011 UART Configuration (`/dev/ttyAMA0`)
Configure the Pi Zero to assign the high-speed PL011 hardware UART to GPIO 14/15 and disable conflicting serial console/Bluetooth services:

```bash
# 1. Enable PL011 UART & disable Bluetooth in boot config
sudo bash -c "cat << 'EOF' >> /boot/firmware/config.txt

[all]
enable_uart=1
dtoverlay=disable-bt
dtparam=i2c_arm=on
EOF"

# 2. Free /dev/ttyAMA0 from Linux serial console & disable Bluetooth services
sudo systemctl disable hciuart.service 2>/dev/null || true
sudo systemctl disable bluetooth.service 2>/dev/null || true
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true

CMDLINE_FILE="/boot/firmware/cmdline.txt"
[ ! -f "$CMDLINE_FILE" ] && CMDLINE_FILE="/boot/cmdline.txt"
sudo sed -i 's/console=serial0,[0-9]\+ //g; s/console=ttyAMA0,[0-9]\+ //g' "$CMDLINE_FILE"

# 3. Add user to dialout & i2c groups
sudo usermod -aG dialout,i2c $USER

# 4. Disable Wi-Fi power saving (prevents high latency spikes and socket timeouts)
sudo mkdir -p /etc/NetworkManager/conf.d/
sudo tee /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf << 'EOF'
[connection]
wifi.powersave = 2
EOF

# 5. Reboot to apply all hardware changes
sudo reboot
```

After the Pi Zero reboots and you reconnect, verify that `/dev/ttyAMA0` is available:
```bash
ls -l /dev/ttyAMA0
# Output should show: crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

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
- 📶 **GNSS Satellite SNR (C/N0) & Constellations (Collapsible)**: Live vertical signal strength bars (in dB-Hz) per satellite across all constellations (GPS, Galileo, BeiDou, GLONASS) with carrier phase lock threshold indicators (≥38 dB-Hz).
- 🔋 **Battery, Power & Thermal Health (Collapsible)**: Live battery %, terminal voltage, dynamic IR-compensated resting EMF, current flow (mA), power (W), CPU temperature, and GPIO 17 button status with safe shutdown timer.
- 📂 **Uncluttered Collapsible Design**: Diagnostic sections feature roll-down headers that default to a compact view with live summary pills (`74% • 3.82V • +260 mA`, `28 Sats • Avg 41.2 dB-Hz`), preserving state across browser refreshes via `localStorage`.
- 📍 **Fixed Reference Coordinates**: Absolute Latitude, Longitude, and Ellipsoidal Height with a direct **Google Maps** link.
- 🔒 **"Lock Now" & Auto-Restore**: Locks current averaged position or automatically restores previous survey coordinates on boot (0 mm drift).
- 📍 **Saved Location Profiles (Presets)**: Save multiple physical benchmark spots (e.g. "Home Yard", "University Roof", "Spot A"). Switch between them with 1 click from the dropdown without recalibrating!
- 📡 **Active Connected Rovers**: Live table of all robots receiving RTCM3 corrections and byte throughput.
- 📋 **Rover Config Snippet**: Instant copyable YAML parameters formatted for `robot_config.yaml`.

### 4.1 Fleet Dashboard One-Click Integration (`📡 Base Station` Button)

You do **not** need to type or remember the Base Station's IP address when monitoring your fleet:
* The Laptop Fleet Dashboard (`tools/fleet_dashboard/app.py`) automatically discovers the Pi Zero on port `8080` (and `2101`) across your local network, dedicated router, or Windows Mobile Hotspot.
* Both the **Fleet Dashboard** (`index.html`) and the **Waypoint Planner** (`waypoint.html`) include a live **`📡 Base Station`** button in the top navigation bar.
* The button features a live connection dot:
  * 🟢 **Green**: Base Station is active and online on the network. Clicking it opens the web dashboard in a new tab.
  * 🔴 **Red**: Base Station is powered off or searching.
* The Waypoint Planner also has a **`📡 Fetch :8080`** button that queries `/api/status` to instantly lock the Cartesian `(0, 0)` origin to the Base Station's survey coordinates with 0 manual typing.

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

