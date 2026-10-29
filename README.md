# ROS 2 Cone Robot Control Workspace

This repository contains the ROS 2 control workspace and hardware interface for a **Modular Differential Drive (Tank / Skid Steering) Robot** powered by a **Raspberry Pi 5** and controlled remotely over Wi-Fi DDS.

The system uses a **Remote-Brain + Onboard State-Driven Motion Primitive Architecture**: high-level mission planning, fleet monitoring, and visualization run on the remote station, while the Raspberry Pi 5 executes **closed-loop, state-driven motion primitives (`STRAIGHT`, `ROTATE`, `ARC`)** using its 50 Hz BNO08x IMU and pluggable distance sensor (`RTK GPS` or `LiDAR Odometry`).

---

## 1. Documentation Index

| Guide | Description |
| :--- | :--- |
| 🏗️ **[Remote-Brain Architecture](documentation/ARCHITECTURE.md)** | Deep dive into the Remote-Brain paradigm, state-driven control loops, topic registry, and single-parameter GPS/LiDAR switching. |
| 🚀 **[Motion Primitive Architecture](documentation/MOTION_PRIMITIVE_ARCHITECTURE.md)** | High-level Action interface (`/execute_motion`), state-based velocity profiling, and dual-rate IMU + GPS/LiDAR feedback. |
| 🍓 **[Raspberry Pi 5 Setup & Wiring Guide](documentation/RPI5_SETUP.md)** | Hardware setup for Pi 5, swap memory, udev GPIO rules, I2C speed, and Cytron MDD10 / BNO08x wiring pinouts. |
| ⚡ **[Raspberry Pi Zero Setup Guide](documentation/RPI_ZERO_SETUP.md)** | Headless Pi Zero OS configuration, PL011 hardware UART (`dtoverlay=disable-bt`), and Wi-Fi optimization. |
| 📡 **[GPS/RTK Setup Guide](documentation/GPS_RTK_SETUP.md)** | Waveshare LC29H(DA) Dual-band GPS/RTK setup, Pi 5 `/dev/ttyAMA0` serial config, NTRIP rover client, and ROS 2 `/fix` topic. |
| 🏰 **[RTK Base Station Setup Guide](documentation/BASE_STATION_SETUP.md)** | Dedicated Raspberry Pi Zero RTK Base Station setup, LC29H(BS) UART configuration, and local NTRIP caster server. |
| 🦊 **[Foxglove Telemetry Guide](documentation/FOXGLOVE_SETUP.md)** | Cross-platform WebSocket telemetry setup (`ws://<PI5_IP>:8765`) and live sensor visualization. |
| 📦 **[Archive / Legacy Guides](documentation/archive/)** | Historical guides including WSL 2 setup (`documentation/archive/WSL2_SETUP.md`) and Wi-Fi hotspot setup (`documentation/archive/rpi_wifi_hotspot_guide.md`). |

---

## 2. Remote-Brain & Unified Motion Architecture

```text
+------------------------------------------+         +--------------------------------------------------+
|          Remote Computer / Brain         |         |        Raspberry Pi 5 (Smart Motion Gateway)     |
|                                          |         |                                                  |
|  - Mission / Waypoint Planner            |         |  - Unified Motion Primitive Controller (50 Hz)   |
|  - 13-Robot Fleet Dashboard & Foxglove   |========>|  - Cytron MDD10 Motor Driver (Deadband + Trim)   |
|  - Dispatches High-Level Goals:          | (Wi-Fi) |  - BNO08x 6-DOF IMU + INA219 Battery (/imu/data) |
|    /execute_motion (Straight/Rotate/Arc) |         |  - Pluggable Position Sensor:                    |
|    /cmd_primitive, /cmd_step             |         |    * GPS Robot:   LC29H RTK GNSS (/fix)          |
|                                          |         |    * LiDAR Robot: YDLIDAR + rf2o (/odom)         |
+------------------------------------------+         +--------------------------------------------------+
                     |                                                        |
                     +----------- ROS DDS Network (ROS_DOMAIN_ID = 42) -------+
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

## 4. Quickstart & Launch Commands

### On Raspberry Pi 5 (SSH):
```bash
# Build workspace:
colcon build --symlink-install --parallel-workers 2
source install/setup.bash

# Option A: Launch GPS Robot (Default — Motors + IMU + RTK GPS + Motion Primitive Controller)
ros2 launch cone_robot_control robot.launch.py robot_type:=gps

# Option B: Launch LiDAR Robot (Motors + IMU + YDLIDAR + rf2o Odom + Motion Primitive Controller)
ros2 launch cone_robot_control robot.launch.py robot_type:=lidar
```

### High-Level Motion Primitive Testing (Action `/execute_motion`):
```bash
# 1. Drive Straight 4.0 meters forward (closed-loop GPS/Odom distance + 50 Hz IMU yaw lock)
ros2 action send_goal --feedback /execute_motion cone_robot_interfaces/action/ExecuteMotion "{motion_type: 0, distance: 4.0, delta_yaw: 0.0, max_velocity: 0.22}"

# 2. Rotate In-Place 90° CCW (Turn Left by +pi/2 radians ~ 1.5708 rad)
ros2 action send_goal --feedback /execute_motion cone_robot_interfaces/action/ExecuteMotion "{motion_type: 1, distance: 0.0, delta_yaw: 1.5708, max_yaw_rate: 1.0}"

# 3. Drive Continuous Arc: 3.0 meters path while turning +45° left (~0.7854 rad)
ros2 action send_goal --feedback /execute_motion cone_robot_interfaces/action/ExecuteMotion "{motion_type: 2, distance: 3.0, delta_yaw: 0.7854, max_velocity: 0.22}"

# Quick Topic Testing via /cmd_primitive (Vector3: x=distance_m, z=delta_yaw_rad):
ros2 topic pub --once /cmd_primitive geometry_msgs/msg/Vector3 "{x: 3.0, y: 0.0, z: 0.7854}"

# Quick Topic Testing via /cmd_step (Vector3: x=distance_cm, z=delta_yaw_deg):
ros2 topic pub --once /cmd_step geometry_msgs/msg/Vector3 "{x: 300.0, y: 0.0, z: 45.0}"
```
