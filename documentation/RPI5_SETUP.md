# Raspberry Pi 5 Setup & Wiring Guide

This guide details system preparation, GPIO driver installation, swap memory configuration, and wiring for driving the **Cytron MDD10 Rev 2.0 Dual DC Motor Driver**, the **Waveshare LC29H(DA) GPS/RTK HAT**, and the **MikroE BNO08x (BNO080/BNO085) IMU** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

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

### Step B: Install ROS 2 Jazzy, GPIO & I2C Libraries
Run the installation commands:

```bash
cd ~/github/ConeRobot  # or your workspace path
bash documentation/scripts/ros2setup.sh

# Install Raspberry Pi 5 GPIO & I2C driver packages
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio python3-smbus i2c-tools python3-pip

# Install Adafruit BNO08x sensor library
pip3 install --break-system-packages adafruit-circuitpython-bno08x

# Run automated YDLIDAR C++ SDK & ROS 2 driver installation script
bash documentation/scripts/install_ydlidar.sh
```

### Step C: Configure I2C Bus Speed (Wire Length Guidelines)
By default, the Raspberry Pi runs I2C at **100 kHz** (Standard Mode). You can adjust this based on your wiring length:

1. Edit boot config:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
2. Configure baud rate:
   * **Short Wires (< 20 cm / jumper cables):** Use **400 kHz** for low-latency fast transfers:
     ```text
     dtparam=i2c_arm=on,i2c_arm_baudrate=400000
     ```
   * **Long Wires (> 30 cm / high electrical noise from motors):** Use **100 kHz** (default) for superior noise immunity and signal integrity:
     ```text
     dtparam=i2c_arm=on,i2c_arm_baudrate=100000
     ```
3. Save (`Ctrl+O`, `Enter`) and exit (`Ctrl+X`), then reboot if modified:
   ```bash
   sudo reboot
   ```

> [!NOTE]
> If you ever experience intermittent I2C bus timeouts or data glitches when running long sensor wires next to motor power cables, switch back to **100 kHz** (`100000`).

### Step D: Configure Permanent Permissions & ROS 2 Environment
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
| **M2A / M2B** | Right DC Motor Terminals | - | Right DC Motor Output |
| **POWER (+/-)** | External Battery (7V–30V) | - | Motor Power Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 3. MikroE BNO080 / BNO085 Click IMU (I2C Mode)

| MikroE Click Pin | Position on Click | Raspberry Pi 5 Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | Left (Pin 7) | **Pin 1** | **3.3V Power** | Clean 3.3V power rail |
| **GND** | Left (Pin 8) or Right (Pin 16) | **Pin 9** | **Ground** | Common Ground |
| **SDA** | Right (Pin 14) | **Pin 3** | **GPIO 2 (SDA)** | I2C Data line |
| **SCL** | Right (Pin 13) | **Pin 5** | **GPIO 3 (SCL)** | I2C Clock line |
| **INT** | Right (Pin 10) | **Pin 7** | **GPIO 4 (INT)** | Hardware Data-Ready Interrupt |
| **RST** | Left (Pin 2) | *(Leave Unconnected)* | *Reset* | Auto power-on reset used |

> [!TIP]
> **Verify I2C Connection:** Run `sudo i2cdetect -y 1`. You should see address `4a` in the grid.

---

## 4. Waveshare LC29H(DA) GPS/RTK HAT (25279) Pinout & Pi 5 Serial Setup

The **Waveshare LC29H(DA) Dual-band GPS/RTK HAT** uses the 40-pin GPIO header for UART communication.

### A. Pin Coexistence (No Conflicts!)

| HAT Function | RPi 5 Pin | GPIO | Conflict? |
| :--- | :--- | :--- | :--- |
| **UART TX** (HAT RX) | Pin 8 | GPIO 14 (TXD0) | ✅ **No Conflict** (Motors use GPIO 12, 13, 24, 25; IMU uses GPIO 2, 3, 4) |
| **UART RX** (HAT TX) | Pin 10 | GPIO 15 (RXD0) | ✅ **No Conflict** |
| **PPS** (Pulse Per Second) | Pin 7 | GPIO 4 | ✅ **No Conflict** |
| **Reset** | Pin 12 | GPIO 18 | ✅ **No Conflict** |

### B. Raspberry Pi 5 Serial Port Path (`/dev/ttyAMA0`)

On **Raspberry Pi 5**, the RP1 chip assigns the 40-pin GPIO header primary UART to **`/dev/ttyAMA0`**.

---

## 5. How to Build & Run on Raspberry Pi 5

### Step 1: Install Dependencies & Build Workspace
```bash
cd ~/github/ConeRobot
git pull origin main

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch the Robot Stack
```bash
ros2 launch cone_robot_control robot.launch.py
```

Expected startup logs:
```text
[INFO] [mdd10_motor_controller]: Cytron MDD10 Motor Controller Node started successfully.
[INFO] [bno08x_node]: BNO08x IMU Node initialized (Rate: 50.0 Hz, Flipped: True, Game Rotation: True)
```