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

All components interface with the Raspberry Pi 5 without pin conflicts:

```text
Raspberry Pi 5 Pinout Allocation:
  - I2C (IMU):    Pins 1 (3.3V), 3 (SDA/GPIO 2), 5 (SCL/GPIO 3), 7 (INT/GPIO 4), 9 (GND)
  - UART (GPS):   Pins 8 (TXD0/GPIO 14), 10 (RXD0/GPIO 15) -> /dev/ttyAMA0
  - Motors (PWM): Pins 32 (PWM1/GPIO 12), 33 (PWM2/GPIO 13), 18 (DIR1/GPIO 24), 22 (DIR2/GPIO 25)
  - LiDAR (USB):  USB 3.0 / USB 2.0 Port -> /dev/ydlidar
```

### A. Cytron MDD10 Rev 2.0 (Dual Motor Controller)

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND** | Pin 6 (or any GND) | GND | **Common Ground** |
| **M1A / M1B** | Left DC Motor | - | Left Motor Terminals |
| **M2A / M2B** | Right DC Motor | - | Right Motor Terminals |
| **POWER (+/-)** | External Battery (7V–30V) | - | Motor Power Supply Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Never connect battery voltage directly to Raspberry Pi GPIO pins!

### B. MikroE BNO080 / BNO085 Click IMU (I2C Mode)

| MikroE Click Pin | Position on Click | Raspberry Pi 5 Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | Left (Pin 7) | **Pin 1** | **3.3V Power** | Clean 3.3V power rail |
| **GND** | Left (Pin 8) or Right (Pin 16) | **Pin 9** | **Ground** | Common Ground |
| **SDA** | Right (Pin 14) | **Pin 3** | **GPIO 2 (SDA)** | I2C Data line |
| **SCL** | Right (Pin 13) | **Pin 5** | **GPIO 3 (SCL)** | I2C Clock line |
| **INT** | Right (Pin 10) | **Pin 7** | **GPIO 4 (INT)** | Data-ready interrupt |
| **RST** | Left (Pin 2) | *(Leave Unconnected)* | *Reset* | Auto power-on reset used |

> [!TIP]
> **Verify I2C Connection:** Run `sudo i2cdetect -y 1`. You should see address `4a` in the grid.

### C. Waveshare LC29H(DA) GPS/RTK HAT (25279)

- Mounted directly to Pi 5 40-pin GPIO header.
- Yellow Jumper Cap: **Position B** (connects HAT UART to GPIO 14/15 -> `/dev/ttyAMA0`).
- For full NTRIP RTK configuration, see [GPS/RTK Setup Guide](GPS_RTK_SETUP.md).

---

## 4. Power Stability & Brownout Prevention

1. **Step-Down Converter Voltage:** Set your 5V 5A DC-DC buck converter output to **5.15V – 5.20V** (official Raspberry Pi spec) to compensate for voltage drop across the USB-C cable under heavy load.

3. **Motor Cable Twisting:** Tightly twist positive and negative wires between the Cytron MDD10 driver and motors to cancel out magnetic interference on the IMU.

---

## 5. Wi-Fi Auto-Connect & Long-Range Antenna Setup

### Option A: One-Command Automated Setup (Recommended)

Run this on your Raspberry Pi 5 to automatically configure Netplan, enable dual-antenna failover, and disable power save:

```bash
sudo bash documentation/scripts/setup_wifi.sh
```
*(Pre-configured with SSID `Bandi` and password `1234445678`)*

---

### Option B: Manual Configuration

If configuring manually, update `/etc/netplan/50-cloud-init.yaml`:

```bash
sudo tee /etc/netplan/50-cloud-init.yaml << 'EOF'
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
      dhcp6: true
  wifis:
    # 1. High-Gain External Antenna (TP-Link TL-WN722N - Priority 1)
    wlan1:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 100
      access-points:
        "Bandi":
          password: "1234445678"
    # 2. Built-in Internal Antenna (Automatic Fallback)
    wlan0:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 600
      access-points:
        "Bandi":
          password: "1234445678"
EOF

sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan apply
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

### A. Monitor Sensors

```bash
# View 3D Orientation & Gyro/Accel (50 Hz)
ros2 topic echo /imu/data

# View 2D Heading Angle (0° to 360°)
ros2 topic echo /imu/heading

# View 360° LiDAR Scan
ros2 topic echo /scan
```

### B. Keyboard Driving (Teleop)

In a separate terminal (or remotely over the network):

```bash
cd ~/github/ConeRobot
source install/setup.bash
ros2 run cone_robot_control teleop_keyboard
```
Press `W`, `A`, `S`, `D` to drive the robot!