# Raspberry Pi 5 Setup, Wiring & Launch Guide

This is the comprehensive hardware configuration, wiring guide, system setup, and launch reference for driving the **Cytron MDD10 Rev 2.0 Dual DC Motor Driver**, the **MikroE BNO08x IMU**, the **Waveshare LC29H(DA) GPS/RTK HAT**, and the **YDLIDAR T-mini Plus LiDAR** on a **Raspberry Pi 5 (1 GB RAM Edition)** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

> [!IMPORTANT]
> **Hardware Constraint: 1 GB RAM Model**
> The robot operates on a **Raspberry Pi 5 with 1 GB of RAM**. Every service, node, and build step must strictly adhere to low-memory design principles:
> - **Headless Execution Only:** Never launch RViz2, Gazebo, or a desktop GUI on the Pi.
> - **Remote-Brain Architecture:** Heavy computing (path planning, SLAM, vision, RViz) is strictly offloaded to the remote PC.
> - **Swap Space:** A 2GB+ swapfile is mandatory to prevent Linux Out-Of-Memory (OOM) killer terminations during compilation or peak operations.
> - **Throttled Compilation:** Colcon builds on the Pi should limit parallel compilation threads (`-j1` or `-j2`).

---

## 1. 1 GB RAM Optimization Guidelines & Rules

To keep the system stable and avoid out-of-memory crashes on the 1 GB RAM board, adhere to the following architecture rules:

| Category | Guideline | Why It Matters |
| :--- | :--- | :--- |
| **GUI & Visualization** | Run **RViz2** & **Foxglove Studio** on remote PC/laptop only | RViz2 consumes 400 MB–1 GB+ RAM, which instantly exhausts Pi 5 1GB memory. |
| **OS Footprint** | Run Ubuntu Server (headless, no X11/Wayland desktop) | Saves ~400 MB RAM compared to Ubuntu Desktop. |
| **Compilation** | Use `colcon build --parallel-workers 2` or `MAKEFLAGS="-j1"` | GCC/Clang can consume 600 MB+ per core when building C++ templates (`rf2o`, `ydlidar`). |
| **Logging & Output** | Suppress high-frequency INFO printouts (e.g. `--log-level WARN`) | Prevents terminal buffer accumulation and excessive stdout memory churn. |
| **ROS 2 QoS Queues** | Set subscription queue depths to 1–5 (e.g., `SensorDataQoS` or `depth=1`) | Prevents unhandled message queues from buffering in RAM if network slows down. |
| **Swap File** | 2 GB Swap with moderate swappiness (`vm.swappiness=60`) | Acts as a safety net against memory spikes. |

---

## 2. System Preparation (Run Once on RPi 5)

### Option 1: Automated All-In-One Setup via SSH (Recommended)

> [!TIP]
> **Setting Up Over SSH via Wi-Fi**:
> When running the setup over an SSH Wi-Fi connection, always run inside **`tmux`**. If your laptop goes to sleep, changes Wi-Fi, or drops the connection, the script will continue running in the background on the Pi without aborting.

```bash
# 1. Connect to your Raspberry Pi 5 over SSH
ssh conerobot@<PI5_IP>

# 2. Create the github folder and clone the repository (First-time setup)
mkdir -p ~/github && cd ~/github
git clone https://github.com/Bandiesabme/ConeRobot.git
cd ConeRobot

# 3. Start a persistent tmux session (guarantees SSH drops won't abort the build)
tmux new -s setup

# 4. Run the interactive setup script:
# Option A: Standard setup (SSID 'Bandi')
bash scripts/setup_robot_rpi5.sh

# Option B: With your custom Wi-Fi network credentials
bash scripts/setup_robot_rpi5.sh "YOUR_WIFI_SSID" "YOUR_WIFI_PASSWORD"

# 5. Reboot when finished to apply all boot & hardware settings:
sudo reboot
```

> **What happens if your Wi-Fi drops or your laptop goes to sleep?**
> The Raspberry Pi continues running the setup script in the background inside `tmux` without interruption.
> 
> **How to resume your session:**
> 1. Reconnect over SSH:
>    ```bash
>    ssh conerobot@<PI5_IP>
>    ```
> 2. Re-attach to your setup terminal:
>    ```bash
>    tmux attach -t setup    # (or simply: tmux a)
>    ```
> 3. Your entire terminal screen, output history, and interactive `[ENTER]` prompt will reappear right where you left off.
> 
> **Useful `tmux` Shortcuts**:
> - **Detach session without closing**: Press `Ctrl + B`, release, then press `D`.
> - **Scroll through previous terminal logs**: Press `Ctrl + B`, release, then press `[` (use Arrow Keys or `Page Up` / `Page Down` to scroll; press `Q` to exit scroll mode).
> - **List active sessions**: `tmux ls`

---

### Option 2: Step-by-Step Manual Setup

If you prefer to run each step manually, follow steps A through D below:

### Step A: Create 2GB Swap Memory (Mandatory for 1GB RAM)
To prevent OOM freezes during `colcon build` or peak runtime:

```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step B: Run Driver Setup Scripts

```bash
cd ~/github/ConeRobot  # or your workspace directory

# 1. Install ROS 2 Jazzy, GPIO, I2C, Foxglove & Sensor Dependencies
bash scripts/ros2setup.sh

# 2. Build and Install YDLIDAR SDK, YDLIDAR ROS 2 Driver & RF2O Laser Odometry Driver
bash scripts/install_ydlidar.sh
```

### Step C: Configure I2C Bus Speed & USB Power Delivery in Boot Config

Edit the Raspberry Pi boot configuration:
```bash
sudo nano /boot/firmware/config.txt
```

Ensure the following settings are present at the bottom:

```text
# 1. Enable full 1.6A USB current draw for LiDAR and sensors (Required for 5V 5A DC-DC step-down converters)
usb_max_current_enable=1

# 2. I2C Bus Speed Configuration
# For short jumper wires (< 20 cm): Use 400 kHz for fast low-latency transfers
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
# For long wires (> 30 cm / close to motor cables): Use 100 kHz standard for noise immunity
# dtparam=i2c_arm=on,i2c_arm_baudrate=100000
```

Save (`Ctrl+O`, `Enter`) and exit (`Ctrl+X`).

### Step D: Configure Device Permissions & Disable Serial Getty

```bash
# 1. Permanent GPIO & I2C device permissions udev rules
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-gpio.rules
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0666"' | sudo tee /etc/udev/rules.d/99-i2c.rules
sudo usermod -aG i2c $USER
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. Disable serial login console on ttyAMA0 (frees UART exclusively for GPS RTK HAT)
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service

# 3. Automatically set ROS 2 environment & domain ID on boot
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 & GPIO Setup" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export GPIOZERO_PIN_FACTORY=lgpio" >> ~/.bashrc
fi

source ~/.bashrc
sudo reboot
```

---

## 3. Hardware Wiring Checklist

All components interface with the Raspberry Pi 5 without pin conflicts, leaving the entire **`SPI0` bus (`Pins 19–26` / `GPIO 7–11`)** plus `GPIO 24 / 25` (`Pins 18 / 22`) 100% free for future SPI peripherals:

```text
Raspberry Pi 5 Pinout Allocation:
  - I2C (IMU + INA219): Pins 1 (3.3V), 3 (SDA/GPIO 2), 5 (SCL/GPIO 3), 9 (GND) [INT/RST left NC]
  - UART (GPS HAT):     Pins 8 (TXD0/GPIO 14), 10 (RXD0/GPIO 15) -> /dev/ttyAMA0 [Pin 7 GPS PPS, Pin 12 Reset]
  - SPI0 (Future Bus):  Pins 19 (MOSI/GPIO 10), 21 (MISO/GPIO 9), 23 (SCLK/GPIO 11), 24 (CE0/GPIO 8), 26 (CE1/GPIO 7) [100% FREE]
  - Motors (2x3 Group): Pins 29 (DIR1/GPIO 5), 31 (DIR2/GPIO 6), 32 (PWM1/GPIO 12), 33 (PWM2/GPIO 13), 30/34 (GND)
  - LiDAR (USB):        USB 3.0 / USB 2.0 Port -> /dev/ydlidar
```

### A. Cytron MDD10 Rev 2.0 (Dual Motor Controller — Grouped on Pins 29–34)

All 5 wires for the Cytron MDD10 (`DIR1`, `DIR2`, `PWM1`, `PWM2`, and `GND`) sit in **one contiguous 2×3 block (`Pins 29–34`)** on the Raspberry Pi 5 header:

| Inner Row (Odd Pins) | Pin # | Pin # | Outer Row (Even Pins) |
| :--- | :---: | :---: | :--- |
| **`DIR1` → `GPIO 5`** | **29** | **30** | **`GND` (Common Ground)** |
| **`DIR2` → `GPIO 6`** | **31** | **32** | **`PWM1` → `GPIO 12`** |
| **`PWM2` → `GPIO 13`** | **33** | **34** | **`GND` *(Spare)*** |

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **DIR1** | Pin 29 | GPIO 5 | Left Motor Direction |
| **DIR2** | Pin 31 | GPIO 6 | Right Motor Direction |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **GND** | Pin 30 (or Pin 34) | GND | **Common Ground** |
| **M1A / M1B** | Left DC Motor | - | Left Motor Terminals |
| **M2A / M2B** | Right DC Motor | - | Right Motor Terminals |
| **POWER (+/-)** | External Battery (7V–30V) | - | Motor Power Supply Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin (`Pin 30` or `Pin 34`) to the Cytron MDD10 GND pin. Never connect battery voltage directly to Raspberry Pi GPIO pins!

### B. MikroE BNO080 / BNO085 Click IMU (I2C Mode - 4 Wires Only)

| MikroE Click Pin | Position on Click | Raspberry Pi 5 Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | Left (Pin 7) | **Pin 1** | **3.3V Power** | Clean 3.3V logic supply |
| **GND** | Left (Pin 8) or Right (Pin 16) | **Pin 9** | **Ground** | Common system ground |
| **SDA** | Right (Pin 14) | **Pin 3** | **GPIO 2 (SDA)** | I2C Data line (shared with INA219) |
| **SCL** | Right (Pin 13) | **Pin 5** | **GPIO 3 (SCL)** | I2C Clock line (shared with INA219) |

> [!NOTE]
> **Why `INT` and `RST` are NOT Connected (4 Wires Total)**:
> - **Reset (`RST`)**: **Not needed**. The BNO08x has an internal hardware Power-On Reset (POR) circuit and onboard pull-up resistors that hold reset HIGH on power-up. If the ROS 2 node ever needs to reset the sensor during initialization, the driver sends a software reset command packet directly over I2C (SHTP protocol). A dedicated hardware reset wire is completely unnecessary.
> - **Interrupt (`INT`)**: **Not needed**. The driver (`bno08x_node.py`) queries the sensor on a deterministic 50 Hz timer loop over standard I2C polling without hardware interrupts. Leaving `INT` disconnected also prevents electrical contention with the Waveshare GPS HAT's 1PPS signal on Pin 7 (GPIO 4).

> [!TIP]
> **Verify I2C Connection:** Run `sudo i2cdetect -y 1`. You should see address `4a` in the grid.

### C. Waveshare LC29H(DA) GPS/RTK HAT (25279)

- Mounted directly to Pi 5 40-pin GPIO header.
- Yellow Jumper Cap: **Position B** (connects HAT UART to GPIO 14/15 -> `/dev/ttyAMA0`).
- For full NTRIP RTK configuration, see [GPS/RTK Setup Guide](GPS_RTK_SETUP.md).

### D. CJMCU-219 / INA219 I2C Battery Voltage Monitor (0-RAM Integration)

The INA219 is used in **pure voltmeter mode** to measure the robot's battery voltage ($0\text{--}26\text{ V}$) without launching an extra process by sharing the I2C bus with the BNO08x IMU in `bno08x_node`.

> [!CAUTION]
> # ⚠️ CRITICAL DANGER: DO NOT CONNECT `VIN-` TO BATTERY NEGATIVE (–)!
> **`VIN-` IS NOT GROUND! `VIN-` IS NOT BATTERY NEGATIVE!**
> 
> On the CJMCU-219 PCB, **`VIN+` and `VIN-` are directly bridged by an onboard $0.1\,\Omega$ shunt resistor**!
> * If you connect **`VIN+` to Battery (+)** AND **`VIN-` to Battery (–)**, you create an **INSTANT DEAD SHORT CIRCUIT** across your battery:
>   $$I = \frac{12\text{ V}}{0.1\ \Omega} = \mathbf{120\text{ Amperes!}}$$
> * The module will **instantly spark, melt PCB traces, catch fire, or destroy the battery!**
> 
> **MANDATORY RULES FOR VOLTMETER MODE:**
> 1. **`VIN-` MUST REMAIN COMPLETELY EMPTY / DISCONNECTED! NEVER CONNECT A WIRE TO `VIN-`!**
> 2. **Battery Positive (+)** connects **EXCLUSIVELY to `VIN+`** (using a single thin wire).
> 3. **Battery Negative (–)** connects **EXCLUSIVELY to `GND`** (Common System Ground with Pi 5).

> [!NOTE]
> **Pure Voltmeter Mode (No Shunt / No Power Loop Interruption)**:
> Since the goal is exclusively measuring battery voltage and charge level, **no external shunt is needed** and **no high-power cables are interrupted**.
> You only connect a single thin voltage sensing wire from the battery positive terminal to the module's `VIN+` pin. The robot's motors and electronics draw current normally through their own heavy wires.

#### 1. Logic & I2C Bus Wiring (RPi 5 Header)

| CJMCU-219 Pin | Raspberry Pi 5 Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **Pin 1** (or Pin 17) | **3.3V Power** | Always use 3.3V rail to protect RPi 5 3.3V GPIO logic |
| **GND** | **Pin 9** (or any GND) | **Common Ground** | Shared system ground with robot and battery (Battery Negative connects here!) |
| **SDA** | **Pin 3** (GPIO 2) | **I2C1 SDA** | Shared line with BNO08x IMU SDA |
| **SCL** | **Pin 5** (GPIO 3) | **I2C1 SCL** | Shared line with BNO08x IMU SCL |

#### 2. Battery Voltage Sense Connection (Single Thin Wire)

```text
Raspberry Pi 5 Header                   CJMCU-219 Breakout
Pin 1 (3.3V) ─────────────────────────► VCC
Pin 9 (GND)  ──────────────┬──────────► GND
                           │
Battery Negative (–) ──────┘ (COMMON GROUND)

Battery Positive (+) ─────────────────► VIN+  (ONLY THIS PIN - 1x thin sense wire)

                                        VIN-  ◄── ⛔ DO NOT CONNECT! LEAVE EMPTY!
```

> [!TIP]
> **Verify I2C Connection:** Run `sudo i2cdetect -y 1`. Both `40` (INA219) and `4a` (BNO08x) will appear in the matrix.
> 
> **Live ROS 2 Battery Stream:**
> ```bash
> ros2 topic echo /battery_state
> ```

---

## 4. Power Stability & Brownout Prevention

1. **Step-Down Converter Voltage:** Set your 5V 5A DC-DC buck converter output to **5.15V – 5.20V** (official Raspberry Pi spec) to compensate for voltage drop across the USB-C cable under heavy load.

3. **Motor Cable Twisting:** Tightly twist positive and negative wires between the Cytron MDD10 driver and motors to cancel out magnetic interference on the IMU.

---

## 5. Wi-Fi Auto-Connect & Long-Range USB Antenna Setup (TP-Link TL-WN722N v2)

### Option A: One-Command Automated Setup (Recommended, Multi-Run Safe)

Run this on your Raspberry Pi 5 to configure Netplan (`systemd-networkd`), pin any USB Wi-Fi dongle (e.g. **TP-Link TL-WN722N v2**) to `wlusb0`, install the continuous failover watchdog (`wifi-failover.service`), disable Wi-Fi power save & USB autosuspend, and prevent ARP flux:

```bash
# Preserves all existing saved SSIDs (or uses default Bandi if none exist):
sudo bash scripts/setup_wifi.sh

# Add or update a Wi-Fi network (keeps previously saved networks too):
sudo bash scripts/setup_wifi.sh "YOUR_SSID" "YOUR_PASSWORD"

# Replace all saved networks with a single SSID:
sudo bash scripts/setup_wifi.sh --replace "YOUR_SSID" "YOUR_PASSWORD"
```

> [!TIP]
> **TP-Link TL-WN722N v2 Hardware Notes:**
> - Plug the adapter into a **black USB 2.0 port** (not a blue USB 3.0 port) to avoid 2.4 GHz / L1 RFI degrading your LC29H GNSS signal.
> - The TL-WN722N v2 uses the **RTL8188EU** chipset (`rtl8xxxu`, supported out of the box in Ubuntu 24.04 with `linux-firmware`) and is **2.4 GHz only**. If your hotspot is 5 GHz only or the dongle loses link for >18 s, `wifi-failover.service` automatically fails back to onboard `wlan0`.
> - Monitor live failover events anytime with: `journalctl -u wifi-failover.service -f`

---

### Option B: Manual Netplan Configuration

If configuring manually (after installing `/etc/systemd/network/10-wifi-usb-dongle.link` so the USB dongle is named `wlusb0`), update `/etc/netplan/50-cloud-init.yaml`:

```bash
sudo tee /etc/netplan/50-cloud-init.yaml << 'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 100
        use-hostname: false
  wifis:
    # 1. High-Gain External USB Antenna (TP-Link TL-WN722N v2 - Priority 1)
    wlusb0:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 200
        use-hostname: false
      access-points:
        "Bandi":
          password: "1234445678"
    # 2. Built-in Internal Antenna (Automatic Fallback)
    wlan0:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 600
        use-hostname: false
      access-points:
        "Bandi":
          password: "1234445678"
EOF

sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan generate && sudo netplan apply
```

---

## 6. How to Build & Run on Raspberry Pi 5 (1GB RAM)

### Step 1: Pull & Build Workspace (Low-Memory Safe)

```bash
cd ~/github/ConeRobot  # or your workspace directory
git pull origin main

# Build using 2 workers to avoid RAM exhaustion on 1GB board:
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### Step 2: Launch the Robot Stack

```bash
# Launch Full Robot Stack (Motors, IMU, GPS, LiDAR, Odometry, Foxglove Bridge)
ros2 launch cone_robot_control robot.launch.py
```

---

## 7. Live Diagnostics & Verification

### A. Automated Full Setup Check (Read-Only)

Run the health checker anytime after setup, reboot, or hardware changes to audit every boot parameter, permission, Wi-Fi failover rule, ROS 2 package, and sensor bus:

```bash
# Full check (with sudo so /etc/netplan/50-cloud-init.yaml is also inspected):
sudo bash scripts/check_rpi5_setup.sh

# Fast software-only check (skips live I2C/UART probes):
bash scripts/check_rpi5_setup.sh --quick
```

### B. Monitor Sensors

```bash
# View 3D Orientation & Gyro/Accel (50 Hz)
ros2 topic echo /imu/data

# View 2D Heading Angle (0° to 360°)
ros2 topic echo /imu/heading

# View 360° LiDAR Scan
ros2 topic echo /scan
```

### C. Keyboard Driving (Teleop)

In a separate terminal (or remotely over the network):

```bash
cd ~/github/ConeRobot
source install/setup.bash
ros2 run cone_robot_control teleop_keyboard
```
Press `W`, `A`, `S`, `D` to drive the robot!
