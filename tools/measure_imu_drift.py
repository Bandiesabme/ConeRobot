#!/usr/bin/env python3
"""
BNO08x IMU Heading & Drift Measurement Tool
---------------------------------------------
Subscribes to `/imu/heading` and `/imu/data` over ROS 2 to measure:
- Baseline starting orientation
- Angular drift over time (handling 0/360 wrap-around)
- Peak drift and drift rate (deg/hour)
- Live gyroscope noise baseline
- Automatically exports results to a timestamped CSV file for plotting.

Usage:
  python3 tools/measure_imu_drift.py
  python3 tools/measure_imu_drift.py --duration 3600 --interval 10
  python3 tools/measure_imu_drift.py --help
"""

import argparse
import csv
import datetime
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


def calculate_angular_diff(current_deg, initial_deg):
    """Calculate shortest angular difference between two angles in degrees (-180 to +180)."""
    diff = (current_deg - initial_deg + 180.0) % 360.0 - 180.0
    return diff


class ImuDriftMeasurer(Node):
    def __init__(self, duration_sec, interval_sec, csv_output_path):
        super().__init__('imu_drift_measurer')
        self.duration_sec = duration_sec
        self.interval_sec = interval_sec
        self.csv_output_path = csv_output_path

        # State variables
        self.start_time = None
        self.initial_heading = None
        self.current_heading = None
        self.last_log_time = 0.0
        self.sample_count = 0
        self.max_abs_drift = 0.0
        self.drift_history = []

        # Gyro statistics
        self.gyro_z_latest = 0.0
        self.gyro_z_samples = []

        # CSV Logging
        self.csv_file = None
        self.csv_writer = None
        if self.csv_output_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.csv_output_path)), exist_ok=True)
            self.csv_file = open(self.csv_output_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'Timestamp_ISO',
                'Elapsed_Sec',
                'Elapsed_Min',
                'Heading_Deg',
                'Drift_Deg',
                'Abs_Drift_Deg',
                'Gyro_Z_rad_s'
            ])
            self.csv_file.flush()

        # Subscribers
        self.heading_sub = self.create_subscription(
            Float32, '/imu/heading', self._heading_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10
        )

        print("\n" + "=" * 78)
        print("  🧭 BNO08x IMU HEADING DRIFT MEASUREMENT BENCHMARK")
        print("=" * 78)
        print(f"  Target Duration : {self.duration_sec / 60.0:.1f} minutes ({self.duration_sec} seconds)")
        print(f"  Log Interval    : Every {self.interval_sec:.1f} seconds")
        if self.csv_output_path:
            print(f"  CSV Log File    : {self.csv_output_path}")
        print("  Condition       : KEEP THE SENSOR / ROBOT COMPLETELY STILL ON A FLAT DESK")
        print("=" * 78 + "\n")
        print("Waiting for first IMU packet on /imu/heading...")

    def _heading_callback(self, msg):
        now = time.time()
        self.current_heading = msg.data

        if self.initial_heading is None:
            self.start_time = now
            self.initial_heading = self.current_heading
            self.last_log_time = now
            print(f"\n[INFO] Reference baseline locked: {self.initial_heading:7.3f}° at {datetime.datetime.now().strftime('%H:%M:%S')}\n")
            print(f"{'Elapsed (min)':<14} | {'Heading (°)':<12} | {'Drift (°)':<12} | {'Peak Drift (°)':<15} | {'Rate (°/hr)':<12}")
            print("-" * 78)
            return

        elapsed = now - self.start_time
        self.sample_count += 1

        # Periodic Display & CSV Logging
        if (now - self.last_log_time) >= self.interval_sec:
            self.last_log_time = now
            elapsed_min = elapsed / 60.0

            drift_deg = calculate_angular_diff(self.current_heading, self.initial_heading)
            abs_drift = abs(drift_deg)
            if abs_drift > self.max_abs_drift:
                self.max_abs_drift = abs_drift

            self.drift_history.append((elapsed, drift_deg))
            drift_rate_hr = (abs_drift / elapsed_min) * 60.0 if elapsed_min > 0 else 0.0

            # Console print
            print(f"{elapsed_min:<14.2f} | {self.current_heading:<12.3f} | {drift_deg:+12.3f} | {self.max_abs_drift:<15.3f} | {drift_rate_hr:<12.2f}")

            # CSV record
            if self.csv_writer:
                self.csv_writer.writerow([
                    datetime.datetime.now().isoformat(),
                    f"{elapsed:.2f}",
                    f"{elapsed_min:.3f}",
                    f"{self.current_heading:.4f}",
                    f"{drift_deg:.4f}",
                    f"{abs_drift:.4f}",
                    f"{self.gyro_z_latest:.6f}"
                ])
                self.csv_file.flush()

        # Check for duration completion
        if elapsed >= self.duration_sec:
            print("\n[INFO] Benchmark target duration reached!")
            raise KeyboardInterrupt

    def _imu_callback(self, msg):
        self.gyro_z_latest = msg.angular_velocity.z
        self.gyro_z_samples.append(self.gyro_z_latest)

    def print_summary(self):
        if self.start_time is None or self.initial_heading is None or self.current_heading is None:
            print("\n[WARN] No IMU data was received during this run.")
            return

        elapsed_total = time.time() - self.start_time
        elapsed_min = elapsed_total / 60.0
        final_drift = calculate_angular_diff(self.current_heading, self.initial_heading)
        rate_hr = (abs(final_drift) / elapsed_min) * 60.0 if elapsed_min > 0 else 0.0

        # Gyro Stats
        avg_gyro_z = sum(self.gyro_z_samples) / len(self.gyro_z_samples) if self.gyro_z_samples else 0.0
        gyro_bias_deg_s = math.degrees(avg_gyro_z)

        print("\n" + "=" * 78)
        print("  📊 BNO08x BENCHMARK FINAL SUMMARY REPORT")
        print("=" * 78)
        print(f"  Total Duration       : {elapsed_min:.2f} minutes ({elapsed_total:.1f} seconds)")
        print(f"  Samples Processed    : {self.sample_count}")
        print(f"  Initial Baseline     : {self.initial_heading:.3f}°")
        print(f"  Final Heading        : {self.current_heading:.3f}°")
        print(f"  Final Net Drift      : {final_drift:+.3f}°")
        print(f"  Maximum Peak Drift   : {self.max_abs_drift:.3f}°")
        print(f"  Calculated Drift Rate: {rate_hr:.2f}° / hour")
        print(f"  Mean Gyro Z Bias     : {gyro_bias_deg_s:+.5f}°/sec ({avg_gyro_z:+.7f} rad/s)")
        if self.csv_output_path:
            print(f"  Data Logged to       : {self.csv_output_path}")
        print("=" * 78 + "\n")

        if self.csv_file:
            self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(
        description="Measure and benchmark BNO08x IMU compass and heading drift over time."
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=3600,
        help="Duration of the benchmark test in seconds (default: 3600 = 1 hour)"
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=10.0,
        help="Interval in seconds between terminal status logs and CSV rows (default: 10.0s)"
    )
    parser.add_argument(
        '--output-csv',
        type=str,
        default=None,
        help="Custom path to save CSV log (default: tools/logs/imu_drift_YYYYMMDD_HHMMSS.csv)"
    )
    args = parser.parse_args()

    # Generate default timestamped CSV path if not specified
    if args.output_csv is None:
        timestamp_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output_csv = os.path.join('tools', 'logs', f'imu_drift_{timestamp_str}.csv')

    rclpy.init()
    node = ImuDriftMeasurer(
        duration_sec=args.duration,
        interval_sec=args.interval,
        csv_output_path=args.output_csv
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()