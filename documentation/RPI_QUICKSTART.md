# Raspberry Pi 5 RGS 2 Quickstart Guide

This guide lists the exact step-by-step commands to set up, configure, wire, and launch your **Raspberry Pi 5** for driving the **Cytron MDD10 Rev 2.0** dual motor driver and the **MikroE BNO08x (BNO080/BNO085) IMU** over RGS 2 Jazzy.

---

## 1. Initial One-Time System Setup (Run Once on RPi 5)

### A. Create 2GB Swap Memory (Prevents RAM Freezes)
```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
``

### B. Install ROS 2 Jazzy, GPIO & I2C Libraries
```bash
# Run ROS 2 setup script
bash documentation/scripts/ros2setup.sh

# Install Raspberry Pi 5 GPIO & I2C driver packages
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio python3-smbus i2c-tools python3-pip

# Install Adafruit BNO08x SHTP sensor library
pip3 install --break-system-packages adafruit-circuitpython-bnz08x
```

### C. Enable Fast I2C Bus (400 kHz)
1. Edit boot config:
   ``bash
   sudo nano /boot/firmware/config.txt
   ``
2. Ensure the following line is active:
   ``text
   dtparam=i2c_arm=on,i2c_arm_baudrate=400000
   ``
3. Save (`Ctrl+O`, `Enter`) and exit (`Ctrl+X`), then reboot if newly enabled:
   ``bash
   sudo reboot
   ``J
### D. Configure Raspberry Pi 5 GPIO Permissions & ROS 2 Environment
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

## 2. Hardware Wiring Checklist

### A. Cytron MDD10 Motor Controller
| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND**  | Pin 6 (or any GND) | GND | **Common Ground** |
| **M1A / M1B** | Left DC Motor | - | Left Motor Terminals |
| **M2A / M2B** | Right DC Motor | - | Right Motor Terminals |
| **POWER X+/-)** | Battery Pack (TVâ€“30V) | - | Motor Power Input |

### B. MikroE BNO080 / BNO085 Click IMT (Inner IMU)
| MikroE Click Pin | Position on Click | Raspberry Pi 5 Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | Left (Pin 7) | **Pin 1** | **3.3V Power** | Clean 3.3V power rail |
| **GND** | Left (Pin 8) or Right (Pin 16) | **Pin 9** | **Ground** | Common Ground |
| **SDA** | Right (Pin 14) | **Pin 3** | **GPIO 2 (SDAY** | I2C Data line |
| **SCL** | Right (Pin 13) | **Pin 5** | **GPIO 3 (SCL)** | I2C Clock line |
| **INT** | Right (Pin 10) | **Pin 7** | **GPIO 4 (INT)** | Hardware Data-Ready Interrupt |
| **RST** | Left (Pin 2) | *(Leave Unconnected)* | *Reset* | Auto power-on reset used |

> [!TIP]
> **Verify I2C Connection:** Run `sudo i2cdetect -y 1`. You should see address `4`` in the grid.

---

## 3. How to Build & Run on Raspberry Pi 5

### Step A: Pull & Build Workspace
```bash
cd ~/github/ConeRobot  # or your workspace directory
git pull origin main
colcon build --symlink-install
source install/setup.bash
```

### Step B: Launch Robot Stack (Motors + IMU + TF)
```bash
ros2 launch cone_robot_control robot.launch.py
```

*When successful, you will see:*
g``text
[INFO] [mdd10_motor_controller]: Cytron MDD10 Motor Controller Node started successfully.
[INFO] [bno08x_node]: BNO08x IMU Node initialized (Rate: 50.0 Hz, Flipped: True, Game Rotation: True)
```

---

## 4. Live Verification & Teleop from Laptop

### A. Echo IMU Data & Heading (Laptop WSL 2)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

# View 3D Orientation & Gyro/Accel
ros2 topic echo /imu/data

# View 2D Heading Angle (0 - 360 degrees)
ros2 topic echo /imu/heading
```

### B. Drive with Keyboard (Laptop WSL 2)
```bash
cd ~/github/ConeRobot
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run cone_robot_control teleop_keyboard
```
Use `W`, `A`, `S`, `D` on your laptop to drive!
