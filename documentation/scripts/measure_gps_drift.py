#!/usr/bin/env python3
"""
==============================================================================
GPS & RTK Precision & Drift Benchmark Tool
==============================================================================
Description:
    Subscribes to /fix (sensor_msgs/msg/NavSatFix) and measures real-time
    stationary drift in centimeters (East, North, and 2D Radial Distance).
    
    Computes:
      - 2D Standard Deviation (Sigma X, Sigma Y in cm)
      - Maximum drift distance (Max error in cm)
      - CEP (Circular Error Probable 50% & 95% 2DRMS)
      - Percentage of samples in RTK FIX vs RTK FLOAT vs 3D FIX

Usage:
    python3 documentation/scripts/measure_gps_drift.py --samples 300
    or
    python3 documentation/scripts/measure_gps_drift.py --duration 60
==============================================================================
"""

import argparse
import math
import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSDriftBenchmark(Node):
    def __init__(self, target_samples: int) -> None:
        super().__init__('gps_drift_benchmark')
        self.target_samples = target_samples
        self.samples: List[Tuple[float, float, float, int]] = []  # (lat, lon, alt, status)
        self.start_time = time.time()

        self.sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.get_logger().info(f"Collecting {self.target_samples} GPS samples from /fix to measure drift...")
        print("\n" + "=" * 65)
        print("  📍 GPS & RTK STATIONARY DRIFT BENCHMARK")
        print("=" * 65)
        print("Keep the robot stationary. Collecting high-precision samples...\n")

    def fix_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        self.samples.append((msg.latitude, msg.longitude, msg.altitude, msg.status.status))
        count = len(self.samples)

        # Real-time progress display
        if count % 10 == 0 or count == self.target_samples:
            sys.stdout.write(f"\rProgress: [{count}/{self.target_samples} samples] ({count/self.target_samples*100:.1f}%)")
            sys.stdout.flush()

        if count >= self.target_samples:
            print("\n")
            self.compute_and_display_results()
            rclpy.shutdown()

    def compute_and_display_results(self) -> None:
        if not self.samples:
            print("No valid GPS samples collected.")
            return

        total_time = time.time() - self.start_time
        lats = [s[0] for s in self.samples]
        lons = [s[1] for s in self.samples]
        alts = [s[2] for s in self.samples]
        statuses = [s[3] for s in self.samples]

        mean_lat = sum(lats) / len(lats)
        mean_lon = sum(lons) / len(lons)
        mean_alt = sum(alts) / len(alts)

        # Conversion factors from degrees to meters at mean latitude
        lat_to_m = 111132.954 - 559.822 * math.cos(2 * math.radians(mean_lat))
        lon_to_m = 111412.84 * math.cos(math.radians(mean_lat))

        # Calculate dx (East) and dy (North) in centimeters relative to mean
        dx_cm = [(lon - mean_lon) * lon_to_m * 100.0 for lon in lons]
        dy_cm = [(lat - mean_lat) * lat_to_m * 100.0 for lat in lats]
        dz_cm = [(alt - mean_alt) * 100.0 for alt in alts]

        radial_errors_cm = [math.sqrt(x**2 + y**2) for x, y in zip(dx_cm, dy_cm)]

        # Statistical metrics
        std_x = math.sqrt(sum(x**2 for x in dx_cm) / len(dx_cm))
        std_y = math.sqrt(sum(y**2 for y in dy_cm) / len(dy_cm))
        std_z = math.sqrt(sum(z**2 for z in dz_cm) / len(dz_cm))
        
        max_drift_cm = max(radial_errors_cm)
        avg_drift_cm = sum(radial_errors_cm) / len(radial_errors_cm)
        
        # 2DRMS (95% confidence radius in 2D)
        drms_2d = 2.0 * math.sqrt(std_x**2 + std_y**2)
        cep_50 = 0.59 * (std_x + std_y)

        # Status breakdown
        rtk_fix_count = statuses.count(NavSatStatus.STATUS_GBAS_FIX)
        dgps_count = statuses.count(NavSatStatus.STATUS_SBAS_FIX)
        standard_count = statuses.count(NavSatStatus.STATUS_FIX)

        print("-" * 65)
        print(f"  Benchmark Duration : {total_time:.1f} seconds ({len(self.samples)} samples)")
        print(f"  Mean Latitude      : {mean_lat:.8f}°")
        print(f"  Mean Longitude     : {mean_lon:.8f}°")
        print(f"  Mean Altitude      : {mean_alt:.3f} m")
        print(f"  Google Maps Link   : https://www.google.com/maps?q={mean_lat:.8f},{mean_lon:.8f}")
        print("-" * 65)
        print("  📊 DRIFT & ACCURACY METRICS (IN CENTIMETERS):")
        print("-" * 65)
        print(f"  • East-West Drift StdDev (σX)  : {std_x:.2f} cm")
        print(f"  • North-South Drift StdDev (σY): {std_y:.2f} cm")
        print(f"  • Vertical Drift StdDev (σZ)   : {std_z:.2f} cm")
        print(f"  • Average Radial Error         : {avg_drift_cm:.2f} cm")
        print(f"  • Maximum Peak Drift           : {max_drift_cm:.2f} cm")
        print(f"  • 50% CEP Radius (50% of time) : < {cep_50:.2f} cm")
        print(f"  • 95% 2DRMS (95% confidence)   : < {drms_2d:.2f} cm")
        print("-" * 65)
        print("  🛰️ FIX QUALITY BREAKDOWN:")
        print("-" * 65)
        print(f"  • RTK FIX / FLOAT (GBAS)       : {rtk_fix_count / len(statuses) * 100:.1f}% ({rtk_fix_count} samples)")
        print(f"  • DGPS (EGNOS / SBAS)          : {dgps_count / len(statuses) * 100:.1f}% ({dgps_count} samples)")
        print(f"  • Standard 3D Fix (SPS)        : {standard_count / len(statuses) * 100:.1f}% ({standard_count} samples)")
        print("=" * 65 + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="GPS & RTK Precision Drift Measurement")
    parser.add_argument('--samples', type=int, default=300, help="Number of /fix samples to collect (default: 300)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GPSDriftBenchmark(target_samples=args.samples)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by user.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
