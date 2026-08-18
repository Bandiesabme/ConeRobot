#!/usr/bin/env python3
"""
==============================================================================
GPS & RTK Precision & Drift Benchmark Tool
==============================================================================
Description:
    Subscribes to /fix (sensor_msgs/msg/NavSatFix) and measures real-time
    stationary drift in centimeters (East, North, and 2D Radial Distance).
    
    Clearly highlights fix status (RTK FIX vs RTK FLOAT vs SPS / NOT FIX)
    and provides separate drift calculations for both all samples and
    true RTK-locked samples.

Usage:
    python3 documentation/scripts/measure_gps_drift.py --samples 300
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
        print("\n" + "=" * 70)
        print("  📍 GPS & RTK STATIONARY DRIFT BENCHMARK")
        print("=" * 70)
        print(f"Collecting {self.target_samples} samples from /fix (Keep robot stationary)...\n")

    def _get_status_name(self, status_code: int) -> str:
        if status_code in [NavSatStatus.STATUS_GBAS_FIX, 4, 5]:
            return "🔒 RTK FIX/FLOAT"
        elif status_code == NavSatStatus.STATUS_SBAS_FIX:
            return "📡 DGPS (SBAS)"
        elif status_code == NavSatStatus.STATUS_FIX:
            return "⚠️ STANDARD 3D FIX (NOT RTK)"
        else:
            return "❌ NO FIX"

    def fix_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        self.samples.append((msg.latitude, msg.longitude, msg.altitude, msg.status.status))
        count = len(self.samples)
        status_name = self._get_status_name(msg.status.status)

        # Real-time progress display with fix quality
        sys.stdout.write(f"\rProgress: [{count}/{self.target_samples} samples] ({count/self.target_samples*100:.1f}%) | Current Status: {status_name}")
        sys.stdout.flush()

        if count >= self.target_samples:
            print("\n")
            self.compute_and_display_results()
            rclpy.shutdown()

    def _calculate_stats(self, sample_subset: List[Tuple[float, float, float, int]]) -> dict:
        if not sample_subset:
            return {}

        lats = [s[0] for s in sample_subset]
        lons = [s[1] for s in sample_subset]
        alts = [s[2] for s in sample_subset]

        mean_lat = sum(lats) / len(lats)
        mean_lon = sum(lons) / len(lons)
        mean_alt = sum(alts) / len(alts)

        lat_to_m = 111132.954 - 559.822 * math.cos(2 * math.radians(mean_lat))
        lon_to_m = 111412.84 * math.cos(math.radians(mean_lat))

        dx_cm = [(lon - mean_lon) * lon_to_m * 100.0 for lon in lons]
        dy_cm = [(lat - mean_lat) * lat_to_m * 100.0 for lat in lats]
        dz_cm = [(alt - mean_alt) * 100.0 for alt in alts]

        radial_errors_cm = [math.sqrt(x**2 + y**2) for x, y in zip(dx_cm, dy_cm)]

        std_x = math.sqrt(sum(x**2 for x in dx_cm) / len(dx_cm))
        std_y = math.sqrt(sum(y**2 for y in dy_cm) / len(dy_cm))
        std_z = math.sqrt(sum(z**2 for z in dz_cm) / len(dz_cm))
        max_drift = max(radial_errors_cm)
        avg_drift = sum(radial_errors_cm) / len(radial_errors_cm)
        drms_2d = 2.0 * math.sqrt(std_x**2 + std_y**2)
        cep_50 = 0.59 * (std_x + std_y)

        return {
            "mean_lat": mean_lat,
            "mean_lon": mean_lon,
            "mean_alt": mean_alt,
            "std_x": std_x,
            "std_y": std_y,
            "std_z": std_z,
            "max_drift": max_drift,
            "avg_drift": avg_drift,
            "drms_2d": drms_2d,
            "cep_50": cep_50,
            "count": len(sample_subset)
        }

    def compute_and_display_results(self) -> None:
        if not self.samples:
            print("No valid GPS samples collected.")
            return

        total_time = time.time() - self.start_time
        statuses = [s[3] for s in self.samples]

        rtk_samples = [s for s in self.samples if s[3] in [NavSatStatus.STATUS_GBAS_FIX, 4, 5]]
        non_rtk_samples = [s for s in self.samples if s[3] not in [NavSatStatus.STATUS_GBAS_FIX, 4, 5]]

        all_stats = self._calculate_stats(self.samples)
        rtk_stats = self._calculate_stats(rtk_samples)

        rtk_count = len(rtk_samples)
        sps_count = len(non_rtk_samples)

        print("-" * 70)
        print(f"  Benchmark Duration : {total_time:.1f} seconds ({len(self.samples)} total samples)")
        print(f"  Mean Position      : {all_stats['mean_lat']:.8f}°, {all_stats['mean_lon']:.8f}° ({all_stats['mean_alt']:.2f} m)")
        print(f"  Google Maps Link   : https://www.google.com/maps?q={all_stats['mean_lat']:.8f},{all_stats['mean_lon']:.8f}")
        print("-" * 70)
        print("  🛰️ FIX QUALITY BREAKDOWN:")
        print("-" * 70)
        print(f"  • 🔒 RTK FIX / FLOAT (GBAS)     : {rtk_count / len(statuses) * 100:.1f}% ({rtk_count} samples)")
        print(f"  • ⚠️ STANDARD 3D FIX (NOT RTK) : {sps_count / len(statuses) * 100:.1f}% ({sps_count} samples)")
        
        if sps_count > 0:
            print(f"\n  ⚠️ NOTE: {sps_count} samples were recorded before/without RTK Lock (Standard 3D SPS),")
            print("     which accounts for large initial GPS wobble in 'All Samples'.")

        print("-" * 70)
        print("  📊 OVERALL DRIFT (ALL SAMPLES INCLUDING NON-RTK):")
        print("-" * 70)
        print(f"  • East-West Drift StdDev (σX)  : {all_stats['std_x']:.2f} cm")
        print(f"  • North-South Drift StdDev (σY): {all_stats['std_y']:.2f} cm")
        print(f"  • Average Radial Error         : {all_stats['avg_drift']:.2f} cm")
        print(f"  • Maximum Peak Drift           : {all_stats['max_drift']:.2f} cm")
        print(f"  • 95% 2DRMS Radius             : < {all_stats['drms_2d']:.2f} cm")

        if rtk_stats:
            print("-" * 70)
            print(f"  🎯 TRUE RTK-ONLY PRECISION ({rtk_stats['count']} RTK-LOCKED SAMPLES):")
            print("-" * 70)
            print(f"  • East-West Drift StdDev (σX)  : {rtk_stats['std_x']:.2f} cm")
            print(f"  • North-South Drift StdDev (σY): {rtk_stats['std_y']:.2f} cm")
            print(f"  • Average Radial Error         : {rtk_stats['avg_drift']:.2f} cm")
            print(f"  • Maximum Peak Drift           : {rtk_stats['max_drift']:.2f} cm")
            print(f"  • 50% CEP Radius (50% of time) : < {rtk_stats['cep_50']:.2f} cm")
            print(f"  • 95% 2DRMS (95% confidence)   : < {rtk_stats['drms_2d']:.2f} cm")

        print("=" * 70 + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="GPS & RTK Precision Drift Measurement")
    parser.add_argument('--samples', type=int, default=300, help="Number of samples to collect (default: 300)")
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
