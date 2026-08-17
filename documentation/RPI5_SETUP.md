# Raspberry Pi 5 Setup & Wiring Guide

This guide details system preparation, GPIO driver installation, swap memory configuration, and wiring for driving the **Cytron MDD10 Rev 2.0 Dual DC Motor Driver** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

---

## 1. System Preparation (Run Once on RPi 5)

### Step A: Create 2GB Swap Memory (Prevents Out-Of-Memory Crashes)
To prevent OOM freezes during `colcon build` on Raspberry Pi 5 models with 1GB/2GB RAM:

```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step B: Install ROS 2 Jazzy & RPi 5 GPIO Libraries
Run the automated installation script:

```bash
cd ~/github/ConeRobot  # or your workspace path
bash documentation/ros2setup.sh

# Install Raspberry Pi 5 RP1 chip GPIO drivers
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio

# Run automated YDLIDAR C++ SDK & ROS 2 driver installation script
bash documentation/scripts/install_ydlidar.sh
```

### Step C: Configure Permanent Permissions & ROS 2 Environment
Set up permanent GPIO udev rules and default environment variables:

```bash
# Permanent GPIO permissions udev rule
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-gpio.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Automatically set ROS 2 environment & domain ID on boot
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 & GPIO Setup" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export GPIOZERO_PIN_FACTORY=lgpio" >> ~/.bashrc
fi

source ~/.bashrc
```

---

## 2. Hardware Wiring Guide: Cytron MDD10 Rev 2.0

The **Cytron MDD10 Rev 2.0** is a dual-channel 10A DC motor driver operating in **PWM + DIR** sign-magnitude mode.

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND** | Pin 6 (or any GND) | GND | **Common Ground** |
| **M1A / M1B** | Left Motor Terminals | - | Left DC Motor Output |
| **M2A / M2B** | Right Motor Terminals | - | Right DC Motor Output |
| **POWER (+/-)** | External Battery (7V–30V) | - | Motor Power Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 3. Waveshare LC29H(DA) GPS/RTK HAT (25279) Pinout & Pi 5 Serial Setup

The **Waveshare LC29H(DA) Dual-band GPS/RTK HAT** uses the 40-pin GPIO header for UART communication.

### A. Pin Coexistence (No Motor Conflicts!)

| HAT Function | RPi 5 Pin | GPIO | Cytron MDD10 Conflict? |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | ✅ **No Conflict** (Motors use GPIO 12, 13, 24, 25) |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | ✅ **No Conflict** |
| **PPS** (Pulse Per Second) | Pin 7 | GPIO 4 | ✅ **No Conflict** |
| **Reset** | Pin 12 | GPIO 18 | ✅ **No Conflict** |

> [!NOTE]
> Your Cytron MDD10 motor drive control pins (**GPIO 12, 13, 24, 25**) do **NOT** conflict with the Waveshare LC29H GPS/RTK HAT (**GPIO 14, 15**). No motor pin modifications are needed!

### B. Raspberry Pi 5 Serial Port Path Difference (`/dev/ttyAMA0`)

- **Pi 4B vs Pi 5 Difference**: On Raspberry Pi 4B, the primary UART is `/dev/ttyS0`. On **Raspberry Pi 5**, the RP1 chip assigns the 40-pin GPIO header primary UART to **`/dev/ttyAMA0`**.
- When configuring `gpsd` (`/etc/default/gpsd`), set:
  ```bash
  DEVICES="/dev/ttyAMA0"
  ```

---

## 3. How to Build & Run on Raspberry Pi 5

### Step 1: Install Dependencies & Build Workspace
```bash
cd ~/github/ConeRobot
git pull origin main

# Automatically install all package.xml dependencies (YDLIDAR driver, tf2, etc.)
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch the Motor Controller Node
```bash
ros2 launch cone_robot_control robot.launch.py
```

Expected startup logs:
```text
[INFO] [mdd10_motor_controller]: Initializing Cytron MDD10 GPIO pins: Left (PWM:12, DIR:24), Right (PWM:13, DIR:25)
[INFO] [mdd10_motor_controller]: Cytron MDD10 Motor Controller Node started successfully.
```
