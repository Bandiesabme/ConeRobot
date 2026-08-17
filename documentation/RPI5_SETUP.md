# Raspberry Pi 5 Setup, Wiring & Launch Guide

This is the comprehensive hardware configuration, wiring guide, system setup, and launch reference for driving the **Cytron MDD10 Rev 2.0 Dual DC Motor Driver**, the **MikroE BNO08x IMU**, the **Waveshare LC29H(DA) GPS/RTK HAT**, and the **YDLIDAR T-mini Plus LiDAR** on a **Raspberry Pi 5** running **Ubuntu 24.04 LTS (ROS 2 Jazzy)**.

---

## 1. System Preparation (Run Once on RPi 5)

### Step A: Create 2GB Swap Memory (Prevents Out-Of-Memory Freezes)
To prevent OOM freezes during `colcon build` on Raspberry Pi 5 models:

```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step B: Install ROS 2 Jazzy, GPIO, I2C & Sensor Libraries

```bash
cd ~/github/ConeRobot  # or your workspace directory
bash documentation/scripts/ros2setup.sh

# Install Raspberry Pi 5 GPIO, I2C & ROS 2 CLI tools
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio python3-smbus i2c-tools python3-pip ros-jazzy-ros2cli-common-extensions ros-jazzy-ros2topic ros-jazzy-ros2node

# Install Adafruit BNO08x IMU library
pip3 install --break-system-packages adafruit-circuitpython-bno08x

# Install YDLIDAR C++ SDK & ROS 2 driver
bash documentation/scripts/install_ydlidar.sh
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

## 2. Hardware Wiring Checklist

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

## 3. Power Stability & Brownout Prevention

1. **Step-Down Converter Voltage:** Set your 5V 5A DC-DC buck converter output to **5.15V – 5.20V** (official Raspberry Pi spec) to compensate for voltage drop across the USB-C cable under heavy load.
2. **Buffer Capacitor:** Add a **1000 µF to 2200 µF (10V–16V)** electrolytic capacitor across the 5V power output to absorb transient current spikes when motors and LiDAR spin up.
3. **Motor Cable Twisting:** Tightly twist positive and negative wires between the Cytron MDD10 driver and motors to cancel out magnetic interference on the IMU.

---

## 4. How to Build & Run on Raspberry Pi 5

### Step 1: Pull & Build Workspace

```bash
cd ~/github/ConeRobot  # or your workspace directory
git pull origin main

colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch the Robot Stack

```bash
# Launch Motors + IMU (Default)
ros2 launch cone_robot_control robot.launch.py

# Launch Motors + IMU + YDLIDAR
ros2 launch cone_robot_control robot.launch.py launch_lidar:=true
```

Expected startup logs:
```text
[INFO] [mdd10_motor_controller]: Cytron MDD10 Motor Controller Node started successfully.
[INFO] [bno08x_node]: BNO08x IMU Node initialized (Rate: 50.0 Hz, Flipped: True, Game Rotation: True)
```

---

## 5. Live Diagnostics & Verification

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