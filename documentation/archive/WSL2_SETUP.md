# WSL 2 (Windows Subsystem for Linux) Setup Guide

This guide details how to set up **Ubuntu (24.04 Noble / 22.04 Jammy)** on **Windows 11 / 10** using **WSL 2**, install **ROS 2**, and configure **Mirrored Networking** so your laptop can publish `/cmd_vel` messages directly to the Raspberry Pi 5 over Wi-Fi.

---

## 1. Install ROS 2 in WSL 2

Open your WSL 2 Ubuntu terminal on Windows and run the automated setup script:

```bash
cd ~/github/ConeRobot  # or your workspace directory
bash documentation/wsl_setup.sh
source ~/.bashrc
```

The script automatically detects your Ubuntu version:
- **Ubuntu 24.04 (Noble)** $\rightarrow$ Installs **ROS 2 Jazzy**
- **Ubuntu 22.04 (Jammy)** $\rightarrow$ Installs **ROS 2 Humble**
- Adds `source /opt/ros/<distro>/setup.bash` and `export ROS_DOMAIN_ID=42` to your `~/.bashrc`.

---

## 2. Configure Mirrored Networking for DDS Discovery

By default, WSL 2 uses NAT networking, which isolates ROS 2 DDS network traffic from your Wi-Fi interface. To allow ROS 2 nodes running on your laptop (in WSL 2) to seamlessly discover and communicate with the Raspberry Pi 5:

### Step 1: Create `.wslconfig` on Windows
Open **Windows PowerShell** (as standard user) and run:

```powershell
notepad $env:USERPROFILE\.wslconfig
```

### Step 2: Add Mirrored Networking Config
Paste the following configuration into the file:

```ini
[wsl2]
networkingMode=mirrored
```

Save and close Notepad.

### Step 3: Restart WSL 2
In PowerShell, restart WSL to apply changes:

```powershell
wsl --shutdown
```

Reopen your WSL 2 Ubuntu terminal. Your WSL environment now shares your laptop's Wi-Fi network adapter directly.

---

## 3. Verify ROS 2 Environment

In your WSL 2 terminal, verify the environment variables:

```bash
echo $ROS_DISTRO     # Should output 'jazzy' or 'humble'
echo $ROS_DOMAIN_ID  # Should output '42'
```

---

## 4. How to Test Local Control in WSL 2 (Mock Mode)

You can build the package and run teleop in mock hardware mode right inside WSL 2 without needing the physical Raspberry Pi 5 powered on:

### Terminal 1 (Mock Motor Controller):
```bash
cd ~/github/ConeRobot
colcon build --symlink-install
source install/setup.bash
ros2 launch cone_robot_control robot.launch.py --ros-args -p mock_hardware:=true
```

### Terminal 2 (Teleop Keyboard):
```bash
cd ~/github/ConeRobot
source install/setup.bash
ros2 run cone_robot_control teleop_keyboard
```

Use `W`, `A`, `S`, `D` to send velocity commands. Terminal 1 will log the motor speed and direction calculations!


flowchart TB
    subgraph Sensors ["Modular Sensor Layer (Inputs)"]
        direction TB
        LIDAR["LiDAR Module\n(Publishes /scan)"]
        GPS["GPS Module\n(Publishes /fix)"]
        IMU["IMU Module\n(Publishes /imu/data)"]
        CAM["Camera Module\n(Publishes /image_raw)"]
    end

    subgraph Processing ["Localization & Processing Layer"]
        TF["Robot State Publisher\n(URDF Frame Transformations)"]
        EKF["Robot Localization (EKF Node)\n(Fuses /fix + /imu + /odom -> /map)"]
    end

    subgraph Actuation ["Actuation Layer (Outputs)"]
        CTRL["mdd10_motor_controller\n(Subscribes /cmd_vel)"]
        MOTORS["Cytron MDD10 + Motors"]
    end

    subgraph Control ["Laptop / Telemetry Layer (Remote)"]
        RVIZ["RViz 2 / Map Display"]
        NAV2["Nav2 / GPS Waypoint Navigation"]
        TELEOP["Teleop Keyboard / Joystick"]
    end

    LIDAR -.->|/scan| NAV2
    GPS -.->|/fix| EKF
    IMU -.->|/imu/data| EKF
    EKF -.->|/odometry/filtered| NAV2
    TELEOP -->|/cmd_vel| CTRL
    NAV2 -->|/cmd_vel| CTRL
    CTRL --> MOTORS
    TF -.->|/tf transforms| RVIZ
