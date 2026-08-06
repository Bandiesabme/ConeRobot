# ROS 2 Cone Robot Control Workspace

This repository contains the ROS 2 control workspace and hardware interface for a **Differential Drive (Tank Steering) Robot** powered by a **Raspberry Pi 5** and controlled via a **Cytron MDD10 Rev 2.0 Dual DC Motor Driver**.

---

## 1. System Architecture & Dev Workflow

```
+------------------------------------+         +--------------------------------------+
|     Windows Laptop (WSL 2 Ubuntu)  |         |        Raspberry Pi 5 (Robot)        |
|  - Edit Python Nodes in VS Code    |         |  - ROS 2 Jazzy (Ubuntu 24.04 Noble)  |
|  - Run Teleop / Testing in WSL 2   |========>|  - Git Pull & colcon build           |
|  - Git Push updates to GitHub      | (Wi-Fi) |  - Cytron MDD10 Driver (PWM+DIR)     |
+------------------------------------+         +--------------------------------------+
                  |                                              |
                  +--- ROS DDS Network (ROS_DOMAIN_ID = 42) -----+
```

---

## 2. Windows Laptop (WSL 2 Ubuntu) Setup

### Step A: Install ROS 2 in WSL 2
Open your WSL 2 Ubuntu terminal on Windows and run:
```bash
bash documentation/wsl_setup.sh
source ~/.bashrc
```
*(Automatically detects Ubuntu 24.04 Noble $\rightarrow$ installs ROS 2 Jazzy, or 22.04 Jammy $\rightarrow$ installs ROS 2 Humble).*

### Step B: Configure WSL 2 Network for RPi 5 Discovery
To allow ROS 2 DDS nodes in WSL 2 on your laptop to discover the Raspberry Pi 5 over Wi-Fi:

1. Open **Windows PowerShell** (as normal user) and run:
   ```powershell
   notepad $env:USERPROFILE\.wslconfig
   ```
2. Paste the following configuration:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
3. Save and close Notepad.
4. Restart WSL in PowerShell:
   ```powershell
   wsl --shutdown
   ```
Now your WSL 2 Ubuntu shares your laptop's Wi-Fi network interface directly, allowing ROS 2 topics to publish/subscribe across laptop and Raspberry Pi seamlessly!

---

## 3. Raspberry Pi 5 Initial Setup & Optimization

### Step A: Create Swap Memory (Crucial for 1GB RAM Pi)
To prevent out-of-memory crashes on your Raspberry Pi 5:
```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step B: Install ROS 2 & Dependencies on Pi 5
Run the Pi setup script on the Raspberry Pi:
```bash
bash documentation/rso2setup.sh
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio
```

---

## 4. Cytron MDD10 Rev 2.0 Wiring Guide

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND**  | Pin 6 (or any GND) | GND | Common Ground |
| **M1A / M1B** | Left Motor Terminals | - | Left DC Motor Output |
| **M2A / M2B** | Right Motor Terminals | - | Right DC Motor Output |
| **POWER (+/-)** | Battery Pack (7V–30V DC) | - | Motor Power Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 5. Quick Testing Instructions

### Option 1: Test Everything Locally in WSL 2 (No RPi needed yet!)

You can test the entire control logic right now inside WSL 2:

1. **Build the package inside WSL 2**:
   ```bash
   cd ~/ros2v1  # or wherever your project directory is located in WSL
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Terminal 1 (Mock Motor Controller)**:
   ```bash
   source install/setup.bash
   ros2 run cone_robot_control mdd10_motor_controller --ros-args -p mock_hardware:=true
   ```

3. **Terminal 2 (Teleop Keyboard)**:
   ```bash
   source install/setup.bash
   ros2 run cone_robot_control teleop_keyboard
   ```
Press `W`, `A`, `S`, `D` in Terminal 2 and observe Terminal 1 logging motor speed and direction calculations!

---

### Option 2: Live Hardware Execution (Laptop WSL 2 $\leftrightarrow$ RPi 5 over Wi-Fi)

1. **On Raspberry Pi 5**:
   ```bash
   cd ~/ros2_ws
   git pull origin main
   colcon build --symlink-install
   source install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 run cone_robot_control mdd10_motor_controller
   ```

2. **On Windows Laptop (WSL 2)**:
   ```bash
   source install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 run cone_robot_control teleop_keyboard
   ```
Now key presses on your laptop drive the physical motors on your Raspberry Pi 5!
