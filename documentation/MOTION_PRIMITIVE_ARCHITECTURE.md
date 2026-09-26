# High-Level Motion Primitive Architecture & Robot-Side Execution Guide

## 1. Executive Summary: Why This Architecture?

In robotic systems with distributed compute (a powerful main computer/brain communicating with a robot hardware gateway over Wi-Fi), roboticists frequently choose between two main control paradigms:

1. **Continuous Velocity Streaming (`cmd_vel`)**: The main computer continuously calculates and streams linear and angular velocities ($v_x, \omega_z$) at 20–50 Hz over the network to the robot motor driver.
2. **High-Level Motion Primitives (Our Chosen Architecture)**: The main computer sends discrete, high-level intent commands (e.g. *Drive 1.0 m*, *Turn 45°*, *Follow Arc: 1.5 m with 30° turn*), while the robot onboard computer generates smooth trajectories, runs the closed-loop feedback controller, and commands the physical motors locally.

---

### Key Reasons for Choosing the Motion Primitive Architecture

#### A. Network Jitter & Wi-Fi Latency Isolation
Wi-Fi connections inherently suffer from packet loss, latency spikes (5 ms jumping to 200+ ms), and interference. In continuous `cmd_vel` streaming:
- Network jitter directly enters the motor control loop, causing stuttering, uneven acceleration, and jerking.
- A tight safety watchdog on the robot (e.g. 200–500 ms) frequently triggers false-positive emergency stops during brief Wi-Fi hiccups.
- **With Motion Primitives**: The fast control loop (20–50 Hz) runs entirely inside the Raspberry Pi 5. The network is only used for sending goals, monitoring progress, and handling preemption/cancellation. If Wi-Fi packets drop for 300 ms, the robot continues executing its smooth local trajectory safely.

#### B. Total Hardware Abstraction & Decoupling (Future-Proofing)
The main computer should remain completely agnostic of low-level mechanical and electrical realities:
- It should **not** care about DC motor PWM, Cytron MDD10 pin assignments, battery voltage sag, track slip, or effective track width calibration.
- The robot acts as an intelligent, self-contained **"Smart Motion Actuator"**.
- In the future, the main computer can be swapped out (e.g. from an Ubuntu laptop to an Nvidia Jetson Orin, a desktop, or an offboard server) without changing a single line of motor or feedback code on the robot.

#### C. Unified Interface for Multiple Robot Types
The broader project involves two different robot platforms:
1. **Tracked LiDAR Robot**: Tracks, no wheel encoders, 2D LiDAR odometry (`rf2o`) + IMU (`BNO08x`).
2. **Wheeled / GPS Robot**: RTK GPS (`LC29H`) + IMU (`BNO08x`).

By standardizing on a high-level **Motion Primitive Action Interface**, both robots expose the **exact same API** to the main computer. The main computer dispatches the exact same command (`DRIVE_ARC`, `DRIVE_STRAIGHT`, `ROTATE_IN_PLACE`) regardless of whether the physical robot underneath is navigating via LiDAR odometry or RTK GPS.

#### D. Compatibility with Waypoint Navigation & Path Following
High-level motion primitives do **not** preclude advanced waypoint following. The standard path-tracking algorithm in robotics—**Pure Pursuit**—works by calculating a circular arc from the robot to a lookahead point on the global path:
$$\text{Curvature } \kappa = \frac{2 \sin(\alpha)}{L_{\text{lookahead}}}$$
A waypoint planner on the main computer simply converts lookahead targets into consecutive `DRIVE_ARC` goals.

---

## 2. Hardware Context & Constraints

The tracked robot platform operates under specific physical and computational constraints:

| Component | Hardware Specification | Role in Motion Execution |
| :--- | :--- | :--- |
| **Compute** | Raspberry Pi 5 (1 GB RAM) | Headless ROS 2 Jazzy; must maintain a strictly lightweight memory footprint (<500 MB base). Heavy C++ frameworks (like full `ros2_control`) are avoided in favor of lean, deterministic nodes. |
| **Motor Driver** | Cytron MDD10 Rev 2.0 | Dual-channel DC driver running in PWM + Direction mode (GPIO 12/24 and 13/25). |
| **Encoders** | **None** | No wheel or motor shaft encoders are present. Motor voltage/PWM does not guarantee deterministic physical speed. |
| **IMU** | MikroE BNO08x (I2C) | 50 Hz high-bandwidth angular velocity ($\omega_z$) and 6-DOF fused Game Rotation Vector (immune to DC motor magnetic fields). |
| **2D LiDAR** | YDLIDAR T-mini Plus | 6 Hz planar laser scans on `/scan`. |
| **Laser Odometry** | `rf2o_laser_odometry` | 6 Hz planar scan-matching odometry running on the Pi, outputting `/odom` (pose $\Delta x, \Delta y$ and linear speed $v_x$). |
| **Battery Monitor** | CJMCU-219 (INA219 over I2C) | Reads real-time LiPo voltage (10.5V to 12.6V), enabling battery voltage feedforward compensation. |

---

## 3. Feedback Strategy Without Wheel Encoders

Without motor encoders, physical motion cannot be measured at the motor shafts. Furthermore, on tracked/skid-steering vehicles, wheel encoders would suffer from significant slip error during turning.

The solution is a **Dual-Rate Complementary Feedback Architecture**:

```text
                  +-------------------------------------------------------------+
                  |               RASPBERRY PI 5 SENSING & FEEDBACK             |
                  |                                                             |
                  |   [BNO08x IMU] ──(50 Hz)──> Angular Velocity (wz)           |
                  |                             Absolute Heading (yaw)          |
                  |                                     │                       |
                  |                                     ▼ (Fast Inner Loop)     |
                  |   [rf2o Laser] ──( 6 Hz)──> Planar Displacement (dx, dy)    |
                  |   Odometry                  Actual Linear Speed (vx)        |
                  |                                     │                       |
                  |                                     ▼ (Distance-to-Go Loop) |
                  |   [INA219 I2C] ──( 1 Hz)──> Battery Bus Voltage (V_batt)    |
                  |                             (Feedforward PWM Scaling)       |
                  +-------------------------------------------------------------+
```

### Sensor Division of Responsibilities

1. **Angular Dynamics (IMU @ 50 Hz)**:
   - Tracks true inertial rotation rate ($\omega_z$) and relative heading ($\Delta\theta$).
   - Completely immune to track slip, wheel drag, and surface friction changes.
   - Provides low-latency feedback for the fast inner steering loop.
2. **Linear Translation (LiDAR Odometry @ 6 Hz)**:
   - Measures true ground displacement relative to external physical geometry.
   - Immune to track slip: if the tracks spin on loose dirt and the robot stalls, `rf2o` correctly measures $\Delta s = 0$.
   - Runs at 6 Hz (166 ms latency), which is sufficient for distance-to-go tracking and trapezoidal deceleration.
3. **Linear Accelerometer Note**:
   - The linear accelerometer on the IMU is **not** integrated for distance. Chassis vibration, pitch/roll tilt on tracks, and centrifugal forces cause open-loop double-integration to diverge in seconds.

---

## 4. Skid-Steering Kinematics & Feedforward Modeling

### The Effective Track Width ($L_{eff}$)
In standard differential drive kinematics:
$$v_L = v - \frac{\omega \cdot L}{2}, \quad v_R = v + \frac{\omega \cdot L}{2}$$

For a tracked chassis, turning requires the tracks to skid and shear laterally against the ground. The Instantaneous Center of Rotation (ICR) shifts outward, making the vehicle behave as if its track width is significantly wider than the physical dimension:
$$L_{eff} = \chi \cdot L_{geometric} \quad (\chi \approx 1.3 \text{ to } 2.0)$$

With $L_{geometric} = 0.290\,\text{m}$, the controller uses an experimentally calibrated $L_{eff}$ for kinematic feedforward calculations.

### Battery Voltage Feedforward Scaling
DC motor speed is proportional to effective terminal voltage:
$$V_{motor} = \text{DutyCycle} \times V_{battery}$$
When a 3S LiPo battery discharges from 12.6V down to 10.5V, a fixed 50% PWM command results in a 17% drop in motor speed. The motor driver node uses the INA219 battery reading to normalize duty cycle commands:
$$\text{DutyCycle}_{actual} = \text{DutyCycle}_{nominal} \times \left(\frac{V_{nominal}}{V_{measured}}\right)$$

---

## 5. Detailed Component Architecture on the Pi

```text
+---------------------------------------------------------------------------------------+
|                                    MAIN COMPUTER                                      |
|                                                                                       |
|   Higher-Level Planner / Waypoint Navigator / Obstacle Detection Node                 |
+───────────────────────────────────────────┬───────────────────────────────────────────+
                                            │
                                            │ ROS 2 Action: /execute_motion
                                            │ Goal: (type, distance, delta_yaw, v_max)
                                            │ Cancel: cancel_goal_async()
                                            │ Feedback: (progress, distance_left, speed)
                                            │ Result: (success, error_code, actual_dist)
                                            │
+───────────────────────────────────────────▼───────────────────────────────────────────+
|                           RASPBERRY PI 5 (Robot Gateway)                              |
|                                                                                       |
|  +─────────────────────────────────────────────────────────────────────────────────+  |
|  | [1] Motion Action Server                                                        |  |
|  |     - Validates goals, handles preemption, cancellations, and status reporting  |  |
|  +────────────────────────────────────────┬────────────────────────────────────────+  |
|                                           │ New validated motion target               |
|                                           ▼                                           |
|  +─────────────────────────────────────────────────────────────────────────────────+  |
|  | [2] Trajectory & Motion Generator (20 Hz)                                       |  |
|  |     - Generates smooth S-curve / trapezoidal v_target(t) and omega_target(t)    |  |
|  |     - Enforces acceleration limits (a_max, alpha_max)                           |  |
|  |     - Ensures smooth curvature transitions to prevent track jerk at t=0         |  |
|  +────────────────────────────────────────┬────────────────────────────────────────+  |
|                                           │ Reference targets: (v_ref, omega_ref)     |
|                                           ▼                                           |
|  +─────────────────────────────────────────────────────────────────────────────────+  |
|  | [3] Dual-Rate Feedback Controller (50 Hz Timer)                                 |  |
|  |     - Fast Heading Loop (50 Hz): PI controller on IMU yaw / yaw rate            |  |
|  |     - Linear Distance Loop (6 Hz): LiDAR odometry distance-to-go tracker        |  |
|  |     - Stuck Detector: Triggers ABORT if commanded PWM > min but v_actual == 0   |  |
|  +────────────────────────────────────────┬────────────────────────────────────────+  |
|                                           │ Desired body velocity: (v_cmd, omega_cmd) |
|                                           ▼                                           |
|  +─────────────────────────────────────────────────────────────────────────────────+  |
|  | [4] Skid-Steer Mixer & MDD10 Motor Driver                                       |  |
|  |     - Maps (v_cmd, omega_cmd) to left/right duty cycle using L_eff              |  |
|  |     - Applies INA219 battery voltage normalization                              |  |
|  |     - Drives GPIO 12/24 and 13/25 via gpiozero                                  |  |
|  |     - Local watchdog timer (stops motors if internal loop halts)                |  |
|  +─────────────────────────────────────────────────────────────────────────────────+  |
|                                                                                       |
+---------------------------------------------------------------------------------------+
```

---

## 6. Motion Primitive Specifications

### 1. `DRIVE_STRAIGHT`
- **Parameters**: `distance` (meters, $+/-$), `max_velocity` (m/s).
- **Execution**:
  - Distance measured via relative `/odom` position delta from `rf2o`.
  - Heading locked to initial IMU heading at motion start.
  - Active 50 Hz IMU yaw-lock PI controller trims left/right motor speeds to eliminate drift.

### 2. `ROTATE_IN_PLACE`
- **Parameters**: `delta_yaw` (radians or degrees, $+/-$), `max_angular_velocity` (rad/s).
- **Execution**:
  - Tracks spin in opposite directions ($v = 0, \omega \ne 0$).
  - Closed-loop feedback purely driven by 50 Hz BNO08x IMU heading.
  - Smooth deceleration ramp as target heading is approached, settling cleanly within tolerance ($\pm 1.5^\circ$).

### 3. `DRIVE_ARC` (Continuous Curved Motion)
- **Parameters**: `distance` (meters, path length $s$), `delta_yaw` (radians, $\Delta\theta$), `max_velocity` (m/s).
- **Mathematical Model**:
  - Curvature: $\kappa = \frac{\Delta\theta}{s}$.
  - Instantaneous angular target: $\omega(t) = v(t) \cdot \kappa$.
  - Trajectory generator applies a smooth ramp up and ramp down to $v(t)$, which simultaneously and proportionally ramps $\omega(t)$.
  - Outer loop monitors distance traveled via LiDAR odometry; inner loop tracks target heading along the arc via IMU.

### 4. `STOP / BRAKE`
- **Parameters**: `immediate` (boolean).
  - `immediate = true`: Instant PWM cutoff (active motor braking).
  - `immediate = false`: Controlled deceleration to zero within maximum deceleration limits ($a_{max}$).

---

## 7. Safety, Stuck Detection & Failure Modes

1. **Obstacle Preemption**:
   - When the main computer detects an obstacle in its sensor field, it sends `cancel_goal_async()`.
   - The Pi cancels the trajectory generator and brings the robot to a controlled stop in $<100\,\text{ms}$, returning `Result.CANCELED`.
2. **Stuck / Stall Detection**:
   - If the controller outputs significant motor effort ($|\text{PWM}| > 0.35$) for $> 0.6\,\text{s}$, but:
     - LiDAR odometry reports $\Delta s < 0.02\,\text{m}$, and
     - IMU reports $\Delta \theta < 1.0^\circ$,
   - The Pi immediately cuts motor power and returns `Result.ABORTED` with `error_code = ERR_STUCK`. This prevents motor burnout and Cytron driver over-current.
3. **Communication Loss Watchdog**:
   - If the ROS 2 Action client drops offline mid-primitive:
     - **Default Policy**: The Pi safely completes the current bounded primitive (e.g. finishes the remaining 30 cm) and halts in an `IDLE` state.
     - **Configurable Heartbeat**: If configured, the Pi can require a heartbeat ping and halt if disconnected for $> 1.0\,\text{s}$.
