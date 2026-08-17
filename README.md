# ROS 2 Cone Robot Control Workspace

This repository contains the ROS 2 control workspace and hardware interface for a **Modular Differential Drive (Tank Steering) Robot** powered by a **Raspberry Pi 5** and controlled remotely via a **Windows Laptop (WSL 2)** over Wi-Fi DDS.

The system uses a **Remote-Brain Offloaded Compute Architecture**: heavy decision-making, navigation, and visualization run on the laptop, while the Raspberry Pi 5 acts as a lightweight **Hardware I/O Gateway** for motors (Cytron MDD10 Rev 2.0) and plug-and-play sensors (LiDAR, GPS, Camera).

---

## 1. Documentation Index

| Guide | Description |
| :--- | :--- |
| 🏗️ **[Remote-Brain Architecture](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/ARCHITECTURE.md)** | Deep dive into the Remote-Brain offloading paradigm, sequence timing loops, topic registry, and modular plugin pattern. |
| 💻 **[WSL 2 Setup Guide](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/WSL2_SETUP.md)** | Step-by-step setup for Windows laptop, mirrored networking configuration (`.wslconfig`), and running `wsl_setup.sh`. |
| 🍓 **[Raspberry Pi 5 Setup Guide](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/RPI5_SETUP.md)** | Hardware setup for Pi 5, swap memory setup, udev GPIO rules, and Cytron MDD10 wiring pinouts. |
| 🦊 **[Foxglove Telemetry Guide](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/FOXGLOVE_SETUP.md)** | Cross-platform WebSocket telemetry setup (`ws://10.42.0.10:8765`), 2D/3D LiDAR point clouds, and teleop joystick controls. |
| 📡 **[GPS/RTK Setup Guide](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/GPS_RTK_SETUP.md)** | Waveshare LC29H(DA) Dual-band GPS/RTK setup, Pi 5 `/dev/ttyAMA0` serial config, NTRIP rover client, and ROS 2 `/fix` topic. |
| ⚙️ **[Development & Kinematics](file:///wsl$/Ubuntu-24.04/home/bandi/github/conerobot/ConeRobot/documentation/DEVELOPMENT.md)** | ROS 2 node architecture, kinematics math formulas, parameter details (`robot_config.yaml`), and mock mode instructions. |

---

## 2. Remote-Brain & Modular Architecture

```text
+------------------------------------------+         +------------------------------------------+
|      Windows Laptop (The Brain)          |         |    Raspberry Pi 5 (Hardware Gateway)     |
|                                          |         |                                          |
|  - Laptop Brain / Decision Node          |         |  - Cytron MDD10 Motor Controller          |
|  - RViz 2 (3D/2D Live Map & Scanning)    |========>|  - LiDAR Driver Node (/scan)             |
|  - Teleop Keyboard Node                  | (Wi-Fi) |  - GPS Driver Node (/fix)                |
|  - Publishes Movement Commands (/cmd_vel)|         |  - Camera Node (/camera/image_raw)       |
+------------------------------------------+         +------------------------------------------+
                     |                                                    |
                     +------- ROS DDS Network (ROS_DOMAIN_ID = 42) -------+
```

---

## 3. Hardware Wiring Quick Reference (Cytron MDD10 Rev 2.0)

| Cytron MDD10 Pin | Raspberry Pi 5 Pin | GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM1** | Pin 32 | GPIO 12 | Left Motor Speed (PWM) |
| **DIR1** | Pin 18 | GPIO 24 | Left Motor Direction |
| **PWM2** | Pin 33 | GPIO 13 | Right Motor Speed (PWM) |
| **DIR2** | Pin 22 | GPIO 25 | Right Motor Direction |
| **GND** | Pin 6 (or any GND) | GND | Common Ground |
| **M1A / M1B** | Left Motor Terminals | - | Left DC Motor Output |
| **M2A / M2B** | Right Motor Terminals | - | Right DC Motor Output |
| **POWER (+/-)** | External Battery (7V–30V DC)| - | Motor Power Input |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 4. Quickstart Testing Commands

### Option A: Local Testing in WSL 2 (Mock Hardware Mode)

1. **Build the package in WSL 2**:
   ```bash
   cd ~/github/ConeRobot
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Terminal 1 (Mock Motor Controller Node)**:
   ```bash
   source install/setup.bash
   ros2 launch cone_robot_control robot.launch.py --ros-args -p mock_hardware:=true
   ```

3. **Terminal 2 (Teleop Keyboard Node)**:
   ```bash
   source install/setup.bash
   ros2 run cone_robot_control teleop_keyboard
   ```
   Press `W`, `A`, `S`, `D` in Terminal 2 to observe motor speed and direction calculations!

---

### Option B: Remote Brain Control (Laptop WSL 2 $\leftrightarrow$ RPi 5 over Wi-Fi)

1. **On Raspberry Pi 5 (Start Hardware Drivers & Motor Controller)**:
   ```bash
   cd ~/github/ConeRobot
   git pull origin main
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch cone_robot_control robot.launch.py
   ```

3. **Visual Telemetry on Windows (Foxglove Studio over WebSockets)**:
   - On Raspberry Pi 5, run Foxglove Bridge:
     ```bash
     ros2 run foxglove_bridge foxglove_bridge
     ```
   - On Windows Laptop, open **[studio.foxglove.dev](https://studio.foxglove.dev)** in Chrome/Edge, click **Open Connection**, and enter `ws://<PI5_IP>:8765`.
   - Add a 2D/3D LaserScan panel for `/scan` or a Teleop panel for `/cmd_vel` to monitor and control your robot live!
