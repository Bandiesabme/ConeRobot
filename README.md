# ROS 2 Cone Robot Control Workspace

This repository contains the ROS 2 control workspace and hardware interface for a **Modular Differential Drive (Tank Steering) Robot** powered by a **Raspberry Pi 5** and controlled remotely over Wi-Fi DDS.

The system uses a **Remote-Brain Offloaded Compute Architecture**: heavy decision-making, navigation, and visualization run on the remote station, while the Raspberry Pi 5 acts as a lightweight **Hardware I/O Gateway** for motors (Cytron MDD10 Rev 2.0) and plug-and-play sensors (LiDAR, IMU, GPS, Camera).

---

## 1. Documentation Index

| Guide | Description |
| :--- | :--- |
| 🏗️ **[Remote-Brain Architecture](documentation/ARCHITECTURE.md)** | Deep dive into the Remote-Brain offloading paradigm, sequence timing loops, topic registry, and modular plugin pattern. |
| 🍓 **[Raspberry Pi 5 Setup & Wiring Guide](documentation/RPI5_SETUP.md)** | Hardware setup for Pi 5, swap memory, udev GPIO rules, I2C speed, and Cytron MDD10 / BNO08x wiring pinouts. |
| 📡 **[GPS/RTK Setup Guide](documentation/GPS_RTK_SETUP.md)** | Waveshare LC29H(DA) Dual-band GPS/RTK setup, Pi 5 `/dev/ttyAMA0` serial config, NTRIP rover client, and ROS 2 `/fix` topic. |
| 🏰 **[RTK Base Station Setup Guide](documentation/BASE_STATION_SETUP.md)** | Dedicated Pi 5 Base Station setup, LC29H(BS) UART configuration, and local NTRIP caster server. |
| 🦊 **[Foxglove Telemetry Guide](documentation/FOXGLOVE_SETUP.md)** | Cross-platform WebSocket telemetry setup (`ws://10.42.0.10:8765`), 2D/3D LiDAR point clouds, and teleop joystick controls. |
| 📦 **[Archive / Legacy Guides](documentation/archive/)** | Historical guides including WSL 2 setup (`documentation/archive/WSL2_SETUP.md`) and Wi-Fi hotspot setup (`documentation/archive/rpi_wifi_hotspot_guide.md`). |

---

## 2. Remote-Brain & Modular Architecture

```text
+------------------------------------------+         +------------------------------------------+
|          Remote Computer / Brain         |         |    Raspberry Pi 5 (Hardware Gateway)     |
|                                          |         |                                          |
|  - Decision / Navigation Node            |         |  - Cytron MDD10 Motor Controller         |
|  - RViz 2 / Foxglove Studio              |========>|  - LiDAR Driver Node (/scan)             |
|  - Teleop Control Node                   | (Wi-Fi) |  - IMU Driver Node (/imu/data)           |
|  - Publishes Movement Commands (/cmd_vel)|         |  - GPS Driver Node (/fix)                |
+------------------------------------------+         +------------------------------------------+
                     |                                                    |
                     +------- ROS DDS Network (ROS_DOMAIN_ID = 42) -------+
```

---

## 3. Hardware Wiring Quick Reference

### Cytron MDD10 Rev 2.0 (Dual Motor Driver)
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

### MikroE BNO08x IMU (I2C)
| MikroE Click Pin | Raspberry Pi 5 Pin | Function |
| :--- | :--- | :--- |
| **3V3** | Pin 1 | 3.3V Power |
| **GND** | Pin 9 | Ground |
| **SDA** | Pin 3 | GPIO 2 (I2C SDA) |
| **SCL** | Pin 5 | GPIO 3 (I2C SCL) |
| **INT** | Pin 7 | GPIO 4 (Data-Ready Interrupt) |

> [!CAUTION]
> **Common Ground**: Always connect the Raspberry Pi 5 GND pin to the Cytron MDD10 GND pin. Do NOT connect battery positive voltage to the RPi 5 GPIO pins!

---

## 4. Quickstart Testing Commands

### On Raspberry Pi 5 (Start Hardware Drivers & Motor Controller):
```bash
cd ~/github/ConeRobot
git pull origin main
colcon build --symlink-install
source install/setup.bash

# Launch Motors + IMU
ros2 launch cone_robot_control robot.launch.py

# Optional: Launch with LiDAR enabled
ros2 launch cone_robot_control robot.launch.py launch_lidar:=true
```

### Visual Telemetry (Foxglove Studio over WebSockets):
1. On Raspberry Pi 5, run Foxglove Bridge:
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```
2. Open **[studio.foxglove.dev](https://studio.foxglove.dev)** in Chrome/Edge, click **Open Connection**, and enter `ws://<PI5_IP>:8765`.
3. Add a 2D/3D LaserScan panel for `/scan` or a Teleop panel for `/cmd_vel` to monitor and drive the robot live!
