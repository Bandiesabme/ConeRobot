# Raspberry Pi 5 ROS 2 Quickstart Guide

This guide lists the exact step-by-step commands to set up, configure, and launch your **Raspberry Pi 5** for driving the **Cytron MDD10 Rev 2.0** dual motor driver over ROS 2 Jazzy.

---

## 1. Initial One-Time System Setup (Run Once on RPi 5)

### A. Create 2GB Swap Memory (Prevents 1GB RAM Freezes)
```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### B. Install ROS 2 Jazzy & RPi 5 GPIO Libraries
```bash
# Run ROS 2 setup script
bash documentation/scripts/ros2setup.sh

# Install Raspberry Pi 5 RP1 chip GPIO driver
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio
```

### C. Configure Raspberry Pi 5 GPIO & ROS 2 Environment
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

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND**  | Pin 6 (or any GND) | GND | **Common Ground** |
| **M1A / M1B** | Left DC Motor | - | Left Motor Terminals |
| **M2A / M2B** | Right DC Motor | - | Right Motor Terminals |
| **POWER (+/-)** | Battery Pack (7V–30V) | - | Motor Power Input |

> [!CAUTION]
> **Common Ground**: Always connect the Cytron MDD10 GND pin to a RPi 5 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 3. How to Build & Run on Raspberry Pi 5

### Step A: Pull & Build Workspace
```bash
cd ~/github/ConeRobot  # or ~/ros2_ws
git pull origin main
colcon build --symlink-install
source install/setup.bash
```

### Step B: Launch Motor Controller Node
```bash
ros2 launch cone_robot_control robot.launch.py
```

*When successful, you will see:*
```text
[INFO] [mdd10_motor_controller]: Initializing Cytron MDD10 GPIO pins: Left (PWM:12, DIR:24), Right (PWM:13, DIR:25)
[INFO] [mdd10_motor_controller]: Cytron MDD10 Motor Controller Node started successfully.
```

---

## 4. How to Drive from Laptop (Windows WSL 2)

In your **WSL 2 Ubuntu terminal** on your laptop (connected to the same Wi-Fi):

```bash
cd ~/github/ConeRobot
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run cone_robot_control teleop_keyboard
```

Use `W`, `A`, `S`, `D` on your laptop to steer the robot live!
