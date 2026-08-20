# 🛰️ ConeRobot 13-Fleet Mission Control Web Dashboard

A lightweight, centralized **Multi-Robot Fleet Monitoring & Live Telemetry Dashboard** designed to monitor up to **13 ConeRobots** simultaneously from your laptop, tablet, or phone.

---

## 🌟 Key Features

1. **Zero Added RAM on Raspberry Pi 5 Robots (0 MB)**:
   - The web server and UI run entirely on your laptop.
   - Robots only broadcast standard lightweight sensor topics over Wi-Fi.

2. **Auto-Discovery (Zero IP Typing)**:
   - Automatically scans your local Wi-Fi network for active robots on Port `8765`.
   - Supports `.local` mDNS hostnames (`conerobot01.local` through `conerobot13.local`).
   - If a robot boots with a different IP address, the dashboard discovers it automatically.

3. **Unified 13-Robot Live GPS Map**:
   - Displays all 13 robots on the same satellite / dark street map.
   - Dynamic directional pins (`R01` to `R13`) rotate with each robot's live IMU heading (0°–360°).
   - Color-coded badges: 🟢 RTK FIX (cm precision), 🟡 Float, 🔴 Lost/Offline.

4. **13 Telemetry Cards Grid**:
   - Status: Active / Driving, Standby / Idle, Offline.
   - IMU Heading, Pitch, Roll.
   - Step Controller progress (`Driving 45/200 cm`, `Turn 45°`).
   - Raspberry Pi CPU Temperature & Wi-Fi Ping Latency.
   - Battery voltage placeholder (ready for I2C battery monitor).

5. **Instant 13-Robot Simulator**:
   - Built-in simulation toggle allows you to test the entire 13-robot dashboard right on your laptop without needing physical robots turned on!

6. **Easily Modifiable Topics (`fleet_config.json`)**:
   - Change ROS 2 topic names anytime from `fleet_config.json` or directly via the **⚙️ Settings** modal in the browser.

---

## 🚀 How to Launch on Your Laptop

### On Windows:
Double-click `run_dashboard.bat`, or run in terminal:
```cmd
cd tools/fleet_dashboard
python app.py
```

### On Linux / macOS:
```bash
cd tools/fleet_dashboard
python3 app.py
```

Then open your browser to:
👉 **[http://localhost:8000](http://localhost:8000)**  
*(Or `http://<your-laptop-ip>:8000` to view from your phone or tablet on the same Wi-Fi!)*

---

## ⚙️ Configuration (`fleet_config.json`)

```json
{
  "server": {
    "port": 8000,
    "robot_port": 8765
  },
  "topics": {
    "gps_fix": "/fix",
    "gps_status": "/gps/status",
    "heading": "/imu/heading",
    "imu_data": "/imu/data",
    "step_status": "/step_status",
    "battery": "/battery_state"
  }
}
```
