# 🦊 Foxglove Studio Setup & Telemetry Guide

This guide explains how to connect **Foxglove Studio** (running on Windows, Linux, or Web Browser) to your **Raspberry Pi 5 ConeRobot** over TCP WebSockets (`port 8765`).

Foxglove Studio provides live 2D/3D LiDAR visualization, sensor diagnostics, and virtual joystick teleop control without needing complex DDS multicast configurations or firewall changes.

---

## 1. Why Foxglove Studio?

- **Cross-Platform**: Runs natively on Windows, Linux, macOS, or in any modern Web Browser (Chrome / Edge / Firefox).
- **TCP WebSocket Streaming**: Communicates over standard HTTP/WebSockets (`ws://10.42.0.10:8765`), completely bypassing Windows Firewall and WSL 2 networking bottlenecks.
- **Rich Visualization**: Supports 2D/3D point clouds, raw message inspectors, live graphs, transform trees, and interactive virtual joysticks.

---

## 2. Raspberry Pi 5 Setup (Server Side)

### Step 1: Install `foxglove_bridge` on Pi 5

Run the following command in a terminal on your Raspberry Pi 5 (`conerobot@conerobot`):

```bash
sudo apt update
sudo apt install -y ros-jazzy-foxglove-bridge ros-jazzy-ros2run
```


### Step 3: Launch Hardware Drivers & Foxglove Bridge

1. **Terminal 1 — Launch Robot Hardware (Motors + LiDAR)**:
   ```bash
   cd ~/github/ConeRobot
   source install/setup.bash
   ros2 launch cone_robot_control robot.launch.py launch_lidar:=true
   ```

2. **Terminal 2 — Start Foxglove WebSocket Bridge**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 run foxglove_bridge foxglove_bridge
   ```

You will see output confirming the server is listening:
```text
[INFO] [foxglove_bridge]: Server listening on port 8765
[INFO] [foxglove_bridge]: Advertising new channel for topic "/scan"
[INFO] [foxglove_bridge]: Advertising new channel for topic "/cmd_vel"
```

---

## 3. Connecting from Laptop / Windows (Client Side)

### Step 1: Open Foxglove Studio

You can use either the Web app or Desktop app:

- **Web Browser (Recommended)**: Open **[https://studio.foxglove.dev](https://studio.foxglove.dev)** in Chrome or Edge.
- **Desktop App**: Download and install [Foxglove Studio Desktop](https://foxglove.dev/download).

### Step 2: Open WebSocket Connection

1. Click **"Open Connection"** (or **"Add Connection"**).
2. Select **Foxglove WebSocket**.
3. Enter your Raspberry Pi 5 IP address:
   ```text
   ws://10.42.0.10:8765
   ```
4. Click **Connect**.

---

## 4. Configuring Visualization Panels

### A. 3D / 2D LiDAR Scan Panel (`/scan`)

1. Click **Add Panel** (top-left `+` button) $\rightarrow$ Select **3D**.
2. On the **left sidebar panel settings**:
   - Under **Frame**: Set **`Display frame`** and **`Fixed frame`** to **`laser_frame`** (or `base_link`).
   - Under **Topics** $\rightarrow$ **`/scan`**:
     - Check the box to enable `/scan`.
     - Set **Point size** to `8.0`.
     - Set **Color mode** to `Gradient` or `Intensity`.
3. Switch between **3D View** and **Top-Down 2D View** using the camera icons on the right side of the canvas.

### B. Raw Message Inspector

1. Click **Add Panel** $\rightarrow$ Select **Raw Messages**.
2. Select topic **`/scan`**.
3. Inspect live numeric range arrays (distances in meters across 360 degrees) and intensity values.

### C. Virtual Joystick Teleop Control (`/cmd_vel`)

1. Click **Add Panel** $\rightarrow$ Select **Teleop**.
2. Set **Publish topic** to **`/cmd_vel`**.
3. Drag the virtual joystick with your mouse on Windows to publish velocity commands directly to the Cytron MDD10 motor controller on the Pi 5!

---

## 5. Troubleshooting & Tips

| Problem | Cause | Solution |
| :--- | :--- | :--- |
| `Peers: unknown element` error on Pi 5 | Invalid `CYCLONEDDS_URI` in `~/.bashrc` | Run `sed -i '/CYCLONEDDS/d' ~/.bashrc` and `unset CYCLONEDDS_URI`. |
| Warning `/move_base_simple/goal` | Default Nav2 goal tool active | Open 3D panel settings $\rightarrow$ Tools $\rightarrow$ Turn off **Publish Goal**. |
| Yellow exclamation mark on `Display frame` | Frame `world` does not exist | Change `Display frame` to `laser_frame` or `base_link`. |
| Connection refused on `8765` | `foxglove_bridge` not running on Pi 5 | Ensure `ros2 run foxglove_bridge foxglove_bridge` is active. |

---

## 6. Quick Reference Commands

```bash
# On Raspberry Pi 5
ros2 launch cone_robot_control robot.launch.py launch_lidar:=true
ros2 run foxglove_bridge foxglove_bridge

# On Windows / Laptop Browser
URL: https://studio.foxglove.dev
WebSocket Address: ws://10.42.0.10:8765
```
