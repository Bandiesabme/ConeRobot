# Navigation, Waypoint Homing & Field Testing Guide

This guide documents the two CLI testing and diagnostic scripts for the ConeRobot motion and navigation stack, along with the underlying dual-sensor (`RTK GPS + LiDAR`), startup compass, and RTK Base Station `(0, 0)` coordinate architecture:

1. **[`scripts/test_action_motion.py`](../scripts/test_action_motion.py)** — Safe Action Runner & 1 Hz Telemetry Logger for motion primitives (`straight`, `rotate`, `circle`, `goto`, `stop`).
2. **[`scripts/test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py)** — Dedicated Waypoint & Sensor Diagnostic Suite (`status`, `offset --return-start`, `base`, `global`).

> [!IMPORTANT]
> **Ctrl+C Safety**: Standard `ros2 action send_goal` leaves the robot driving if you press `Ctrl+C` in the terminal. Both [`test_action_motion.py`](../scripts/test_action_motion.py) and [`test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py) trap `KeyboardInterrupt` and transmit an immediate `cancel_goal_async()` request to `/execute_motion` so the motors stop the instant you press `Ctrl+C`.

---

## 1. Navigation & Sensor Fusion Architecture

### 1.1 Shared Fleet Origin: RTK Base Station as `(0, 0)`
To allow a 14-robot fleet to share the exact same metric coordinate system without manual surveying:
- **[`lc29h_gps_node.py`](../src/cone_robot_control/cone_robot_control/lc29h_gps_node.py)** inspects incoming NTRIP RTCM3 packets (`0xD3` framing) and decodes **RTCM Message 1005 / 1006** (*Stationary RTK Reference Station ARP*).
- The 38-bit ECEF `(X, Y, Z)` base station coordinates are converted to WGS84 `(lat, lon, alt)` and published on `/gps/base_fix` (`sensor_msgs/NavSatFix`).
- Every robot automatically defines the RTK Base Station antenna as **`(0.0 m East, 0.0 m North)`** (`+X` = East, `+Y` = North in meters).
- You can also override the `(0, 0)` origin manually via `base_lat` and `base_lon` in [`robot_config.yaml`](../src/cone_robot_control/config/robot_config.yaml).

### 1.2 Startup 25-Sample Compass Calibration + 6-DOF Gyro + RTK COG
Tracked skid-steer robots have high DC motor currents and steel hardware that can distort magnetometers during driving. To determine global heading with **`0.0 m` of initial movement** while remaining immune to magnetic interference:
1. **Startup Stationary Averaging (Once at Boot)**:
   - [`bno08x_node.py`](../src/cone_robot_control/cone_robot_control/bno08x_node.py) publishes the 9-DOF magnetometer-referenced heading on `/imu/compass_heading` alongside the 50 Hz 6-DOF `Game Rotation Vector` on `/imu/heading`.
   - At node startup, [`primitive_motion_controller.py`](../src/cone_robot_control/cone_robot_control/primitive_motion_controller.py) skips the first **5 warmup samples** (while the BNO08x internal filter settles) and circularly averages (`atan2(sum_sin, sum_cos)`) the next **25 stationary samples (~1.5 seconds)**.
   - Once those 25 samples are collected, `self.compass_initialized = True` is locked and **the magnetometer is ignored for the rest of the session**. All turns (`ROTATE`, `ARC`, `GOTO ALIGN`) use the drift-free 50 Hz 6-DOF gyro.
2. **True Geographic RTK Course-Over-Ground (`COG`) Calibration**:
   - On the first straight segment (`>= 20 cm` traveled with `RTK FIX`), [`primitive_motion_controller.py`](../src/cone_robot_control/cone_robot_control/primitive_motion_controller.py) computes the true geographic movement vector `atan2(dNorth, dEast)` from RTK GPS.
   - This locks `self.enu_yaw_locked_by_rtk = True`, removing any local magnetic declination or frame iron offset so the robot steers directly onto the target coordinate.

### 1.3 Dual-Sensor (`GPS + LiDAR`) Auto-Handover (`distance_source: "auto"`)
When launched with `robot_type:=all` (default in [`robot.launch.py`](../src/cone_robot_control/launch/robot.launch.py)):
- **Outdoors with `RTK FIX`**: The controller uses survey-grade RTK GPS (`/fix`) combined with a 50 Hz sub-second velocity predictor (`_gps_interfix_dist_m`) and a stationary `/fix` confirmation gate at the end of the motion.
- **Last-RTK-Fix LiDAR Handover (`RTK FIX` $\rightarrow$ `RTK FLOAT` / Indoors)**:
  - On every valid `RTK FIX` sample, [`_fix_callback`](../src/cone_robot_control/cone_robot_control/primitive_motion_controller.py) snapshots `self._dist_at_last_rtk_fix` and the simultaneous LiDAR odometry pose `self._odom_pos_at_last_rtk_fix`.
  - If GPS degrades to `RTK FLOAT` (e.g. under trees or entering a building), the degraded Float coordinate is immediately rejected (preventing a 0.5–1.5 m position jump), and the controller seamlessly measures the remaining distance from the **exact LiDAR pose captured at the last valid `RTK FIX`**.
  - If `RTK FIX` recovers before the motion completes, the controller switches back to `GPS`.
- **LiDAR-Only Robots (`robot_type:=lidar`)**:
  - Both [`scripts/test_action_motion.py`](../scripts/test_action_motion.py) and [`scripts/test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py) subscribe to `/odom` (`nav_msgs/Odometry`) in addition to `/fix`.
  - When `/fix` is absent, both scripts automatically display `Sensor: [🟣 LIDAR ODOM]`, record the 1 Hz step-by-step telemetry table from `/odom`, and evaluate all displacement, circle closure, and `(X, Y)` waypoint benchmarks in the LiDAR `/odom` frame (where `(0, 0)` is the robot's startup pose).

---

## 2. Script 1: [`scripts/test_action_motion.py`](../scripts/test_action_motion.py)

[`scripts/test_action_motion.py`](../scripts/test_action_motion.py) is the primary CLI runner for testing all motion primitives on `/execute_motion` (`STRAIGHT`, `ROTATE`, `ARC`, `GOTO_BASE_XY`, `GOTO_REL_ENU`, and `STOP`).

### 2.1 Subcommands & Examples

#### A. Drive Straight (`straight`) — Forward or Reverse
```bash
# Drive 4.0 meters forward at 0.25 m/s
python3 scripts/test_action_motion.py straight --distance 4.0 --speed 0.25

# Drive 4.0 meters in reverse at 0.25 m/s
python3 scripts/test_action_motion.py straight --distance -4.0 --speed 0.25
```

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--distance` | `1.0` | Target displacement in meters (`+` forward, `-` reverse). |
| `--speed` | `0.25` | Maximum linear velocity in `m/s`. |

#### B. Rotate In-Place (`rotate`)
```bash
# Turn 90 degrees left (CCW) at 1.0 rad/s
python3 scripts/test_action_motion.py rotate --deg 90.0 --rate 1.0

# Turn 90 degrees right (CW) at 1.0 rad/s
python3 scripts/test_action_motion.py rotate --deg -90.0 --rate 1.0
```

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--deg` | `90.0` | Target turn angle in degrees (`+` left/CCW, `-` right/CW). |
| `--rate` | `1.0` | Maximum angular yaw rate in `rad/s`. |

#### C. Full 360° Circular Arc (`circle`)
```bash
# Drive a 1.0m radius circle to the left at 0.20 m/s
python3 scripts/test_action_motion.py circle --radius 1.0 --speed 0.20 --direction left
```

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--radius` | `1.0` | Circle radius in meters (path length $s = 2\pi r$). |
| `--speed` | `0.20` | Linear tangential speed in `m/s`. |
| `--direction` | `left` | Turn direction (`left` for CCW, `right` for CW). |

#### D. Navigate to 2D Waypoint (`goto`)
Supports **both** Base-Station `(0, 0)` coordinates in meters (`--x --y`) and Global WGS84 coordinates (`--lat --lon`):
```bash
# Option 1: Navigate to (X_East = +4.0m, Y_North = +2.5m) relative to RTK Base Station (0,0)
python3 scripts/test_action_motion.py goto --x 4.0 --y 2.5 --speed 0.25

# Option 2: Navigate to a Global WGS84 (Lat, Lon) coordinate
python3 scripts/test_action_motion.py goto --lat 47.4484520 --lon 18.7532365 --speed 0.25
```

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--x` / `--y` | `None` | Target `(East_m, North_m)` relative to RTK Base Station `(0, 0)` (`motion_type=4`). |
| `--lat` / `--lon` | `None` | Target Global WGS84 `(Latitude, Longitude)` in decimal degrees (`motion_type=5`). |
| `--speed` | `0.25` | Maximum linear velocity in `m/s`. |
| `--rate` | `0.8` | Maximum alignment turn rate in `rad/s`. |

#### E. Emergency Stop (`stop`)
```bash
python3 scripts/test_action_motion.py stop
```

### 2.2 Understanding the 1 Hz Telemetry & Motor Cut-Off Table

After every `straight` run, [`scripts/test_action_motion.py`](../scripts/test_action_motion.py) prints a detailed breakdown of every 1 Hz `/fix` update and motor cut-off event:

```text
==============================================================================
 📊 1 Hz RTK GPS STEP-BY-STEP TELEMETRY & MOTOR CUT-OFF LOG
==============================================================================
  # |  t (s) |  GPS Dist |     Step |   True V |  AvgCmdV |  NowCmdV | Scale
------------------------------------------------------------------------------
  1 |  1.00s |    0.182m |  +18.2cm |  0.18m/s |  0.14m/s | +0.25m/s | 1.29x
  2 |  2.00s |    0.514m |  +33.2cm |  0.33m/s |  0.25m/s | +0.25m/s | 1.33x
  ...
 13 | 13.00s |    4.021m |   +4.1cm |  0.04m/s |  0.03m/s | +0.00m/s | 1.31x
------------------------------------------------------------------------------
 🛑 Motor Cut #1 at t=12.42s (+0.42s after Fix@3.980m, last CmdV=+0.052m/s)
==============================================================================
```

- **`GPS Dist` & `Step`**: Cumulative distance from start and incremental step (`cm`) measured between consecutive 1 Hz RTK fixes.
- **`True V` vs `AvgCmdV` & `Scale`**: Actual ground speed (`m/s`) vs commanded `/cmd_vel` speed (`m/s`), showing the empirical motor speed scale (`True V / AvgCmdV`) that the controller learns online (`_gps_speed_scale`).
- **`🛑 Motor Cut`**: Logs the exact sub-second timestamp when `/cmd_vel` dropped to `0.00 m/s`, how many seconds after the previous GPS fix the controller cut power, and the final commanded crawl velocity.
- **`Measured Stationary (+1s)`**: After the action server reports completion, the script spins for `+1.15 s` to capture one additional stationary `/fix` and reports `Post-done drift` in `cm` (isolating physical coasting vs LC29H receiver filter lag).

---

## 3. Script 2: [`scripts/test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py)

[`scripts/test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py) is a specialized diagnostic and field-testing tool designed to verify the startup compass, RTK Base `(0, 0)` decoding, and 2D waypoint homing.

### 3.1 Mode 1: `status` (Zero-Movement Sensor & Coordinate Diagnostic)
Run this first before driving to verify that the robot has `RTK FIX`, has decoded the Base Station `(0, 0)` coordinate from RTCM 1005/1006, and has locked its 25-sample startup compass heading:

```bash
python3 scripts/test_waypoint_navigation.py status
```

**What it outputs:**
- Current RTK status (`RTK FIX`, satellite count, HDOP).
- Robot's current **Global `(Lat, Lon)`** coordinate.
- Decoded **RTK Base Station `(0, 0)` `(Lat, Lon)`** coordinate and the robot's current **`X_East (m), Y_North (m)`** position relative to the base station.
- Current 6-DOF gyro angle (`/imu/heading`) and the circularly averaged **25-sample Startup Compass Heading** in ENU degrees (`0° = East`, `+90° = North`, `±180° = West`, `-90° = South`).
- Ready-to-copy-paste test commands tailored to the robot's current standing location.

### 3.2 Mode 2: `offset` (Relative East/North Offset + Automatic Return-to-Start)
The easiest way to test 2D waypoint navigation in the field without typing coordinates manually. It reads the robot's current position, computes a target `(--east, --north)` meters away, drives there, and (with `--return-start`) pauses 2 seconds and drives back to the exact starting coordinate:

```bash
# Drive +3.0m East and +2.0m North in Base (0,0) coordinates, then return to start:
python3 scripts/test_waypoint_navigation.py offset --east 3.0 --north 2.0 --speed 0.25 --return-start

# Same test using Global (Lat, Lon) waypoint mode:
python3 scripts/test_waypoint_navigation.py offset --east 3.0 --north 2.0 --frame global --speed 0.25 --return-start
```

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--east` | `2.0` | Offset East (`+`) or West (`-`) in meters from current position. |
| `--north` | `0.0` | Offset North (`+`) or South (`-`) in meters from current position. |
| `--frame` | `base` | Coordinate system to test: `base` (`motion_type=4`) or `global` (`motion_type=5`). |
| `--speed` | `0.25` | Max linear speed in `m/s`. |
| `--rate` | `0.8` | Max turn yaw rate in `rad/s`. |
| `--return-start` | `False` | When set, automatically navigates back to the exact starting coordinate after reaching the target. |

### 3.3 Mode 3: `base` (Navigate to RTK Base Station `(0, 0)` Meter Coordinates)
Send any robot in the 14-robot fleet to a specific `(X_East, Y_North)` meter coordinate relative to the RTK Base Station `(0, 0)`:

```bash
python3 scripts/test_waypoint_navigation.py base --x 10.50 --y -3.20 --speed 0.25
```

### 3.4 Mode 4: `global` (Navigate to Global WGS84 `(Lat, Lon)` Coordinates)
Navigate to an exact 7-decimal-place WGS84 latitude and longitude:

```bash
python3 scripts/test_waypoint_navigation.py global --lat 47.4484520 --lon 18.7532365 --speed 0.25
```

### 3.5 Reading the Waypoint Telemetry & Arrival Benchmark
During and after each waypoint run, [`scripts/test_waypoint_navigation.py`](../scripts/test_waypoint_navigation.py) displays:
1. **Live Progress Bar**:
   - Shows the active phase (`ALIGN` while rotating in place toward the waypoint bearing, `DRIVE` while driving and continuously homing onto the target).
   - Shows `HdgSrc: Compass` initially, switching to `HdgSrc: COG:+XX°` as soon as the robot has driven `20 cm` and locked its true geographic Course-Over-Ground from RTK GPS.
2. **1 Hz Waypoint Telemetry Log**:
   - Logs `#`, `t (s)`, `Base X (E)`, `Base Y (N)`, `Cmd V`, `Cmd W`, `Compass` heading (`° ENU`), and `RTK COG` heading (`° ENU`) at every 1 Hz GPS update.
3. **2D Waypoint Arrival Benchmark**:
   - Compares the commanded target against the final settled RTK position in both `(X_East, Y_North)` meters and `(Lat, Lon)` degrees, reporting the radial **2D Arrival Error in `cm`**.
