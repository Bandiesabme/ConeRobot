# Diagnostics & Calibration Tools

This directory contains standalone diagnostics, benchmarking, and calibration utilities for the **ConeRobot** platform.

---

## 1. IMU Heading & Gyro Drift Benchmark (`measure_imu_drift.py`)

This tool connects to the live ROS 2 IMU stream (`/imu/heading` and `/imu/data`) to measure long-term heading stability, angular drift, and gyroscope bias over time.

### Features
* Locks onto the initial heading baseline at $t=0$.
* Calculates real-time angular drift ($|\Delta \theta|$) handling $0^\circ / 360^\circ$ wrap-around.
* Computes real-time drift rate in degrees per hour ($^\circ/\text{hr}$).
* Measures mean gyroscope Z-axis bias ($\text{rad/s}$ and $^\circ/\text{sec}$).
* Automatically saves a timestamped CSV log in `tools/logs/` for easy graphing in Excel, Python, or MATLAB.
* Prints a summary statistical report upon completion or when stopped with `Ctrl+C`.

---

### How to Run

Make sure the robot drivers are running in another terminal (`ros2 launch cone_robot_control robot.launch.py`).

#### A. Standard 1-Hour Benchmark Test (Default: 3600 seconds, 10s log interval):
```bash
python3 tools/measure_imu_drift.py
```

#### B. Quick 5-Minute Test (300 seconds, 5s interval):
```bash
python3 tools/measure_imu_drift.py --duration 300 --interval 5
```

#### C. Custom Output CSV Location:
```bash
python3 tools/measure_imu_drift.py --duration 1800 --output-csv my_test_results.csv
```

---

### Expected Results

| Mode | Configuration | Expected 1-Hour Drift |
| :--- | :--- | :--- |
| **Game Rotation Vector (6-DOF)** | `use_game_rotation: true` | $\approx 1.0^\circ - 3.0^\circ$ / hour |
| **Standard Rotation Vector (9-DOF)** | `use_game_rotation: false` | $\approx 0.0^\circ$ / hour (Locked to Magnetic North) |
