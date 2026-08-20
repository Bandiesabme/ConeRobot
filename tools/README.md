# 🛠️ Diagnostics, Benchmarking & Fleet Tools

This directory contains standalone diagnostics, benchmarking, calibration, and fleet management utilities for the **ConeRobot** platform.

---

## 1. GPS & RTK Precision Benchmark (`measure_gps_drift.py`)

This tool connects to the live ROS 2 GPS stream (`/fix`) to benchmark real-time stationary position drift in centimeters (East, North, and 2D Radial Error).

### Features
* **Fix Quality Breakdown**: Accurately categorizes and benchmarks **RTK FIX** ($1\text{--}2\text{ cm}$), **RTK FLOAT** ($20\text{--}100\text{ cm}$), and standard **3D SPS** ($2\text{--}5\text{ m}$) fixes.
* **Statistical Dispersion**: Calculates 2D Radial Error Mean, Median, Standard Deviation ($\sigma$), and Maximum Error.
* **Auto-Conversion**: Converts WGS-84 geodesic latitude/longitude into local metric Cartesian coordinates.
* **Console Summary**: Displays a clean ASCII terminal summary and progress bar.

### How to Run

Ensure the robot drivers or GPS node is running (`ros2 launch cone_robot_control robot.launch.py`):

```bash
# Standard 300-sample benchmark
python3 tools/measure_gps_drift.py --samples 300

# Continuous 1000-sample high-resolution benchmark
python3 tools/measure_gps_drift.py --samples 1000
```

---

## 2. IMU Heading & Gyro Drift Benchmark (`measure_imu_drift.py`)

This tool connects to the live ROS 2 IMU stream (`/imu/heading` and `/imu/data`) to measure long-term heading stability, angular drift, and gyroscope bias over time.

### Features
* Locks onto the initial heading baseline at $t=0$.
* Calculates real-time angular drift ($|\Delta \theta|$) handling $0^\circ / 360^\circ$ wrap-around.
* Computes real-time drift rate in degrees per hour ($^\circ/\text{hr}$).
* Measures mean gyroscope Z-axis bias ($\text{rad/s}$ and $^\circ/\text{sec}$).
* Automatically saves a timestamped CSV log in `tools/logs/` for graphing in Excel, Python, or MATLAB.
* Prints a summary statistical report upon completion or when stopped with `Ctrl+C`.

### How to Run

```bash
# Standard 1-Hour Benchmark Test (Default: 3600 seconds, 10s log interval):
python3 tools/measure_imu_drift.py

# Quick 5-Minute Test (300 seconds, 5s interval):
python3 tools/measure_imu_drift.py --duration 300 --interval 5

# Custom Output CSV Location:
python3 tools/measure_imu_drift.py --duration 1800 --output-csv my_test_results.csv
```

### Expected Results

| Mode | Configuration | Expected 1-Hour Drift |
| :--- | :--- | :--- |
| **Game Rotation Vector (6-DOF)** | `use_game_rotation: true` | $\approx 1.0^\circ - 3.0^\circ$ / hour |
| **Standard Rotation Vector (9-DOF)** | `use_game_rotation: false` | $\approx 0.0^\circ$ / hour (Locked to Magnetic North) |

---

## 3. Web Fleet Management Dashboard (`fleet_dashboard/`)

A high-performance, real-time web monitoring and teleoperation application for managing multiple Cone Robots.

* **Path**: [`tools/fleet_dashboard/`](file:///c:/Users/Bandi/Egyetem/cone%20robot/ros2v1/tools/fleet_dashboard)
* **Features**: Live Leaflet satellite map, real-time robot telemetry, battery/speed gauges, waypoints management, and emergency stop.
* **Launch**:
  ```bash
  cd tools/fleet_dashboard
  python3 app.py --port 5000
  ```

