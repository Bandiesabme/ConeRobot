#!/usr/bin/env python3
"""
==============================================================================
GPS & RTK Precision & Drift Benchmark Tool
==============================================================================
Description:
    Subscribes to /fix (sensor_msgs/msg/NavSatFix) and measures real-time
    stationary drift in centimeters (East, North, and 2D Radial Distance).
    
    Strictly separates and benchmarks:
      1. 🎯 RTK FIX (Survey-Grade 1–2 cm)
      2. ⏳ RTK FLOAT (Sub-meter ~20–100 cm)
      3. ⚠️ Standard 3D SPS (Uncorrected ~2–5 m)
      4. 📊 Combined Overall Benchmark

Usage:
    python3 tools/measure_gps_drift.py --samples 300
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
        # Each entry: (lat, lon, alt, fix_type_str)
        self.samples: List[Tuple[float, float, float, str]] = []
        self.start_time = time.time()
        self.is_completed = False

        self.sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        print("\n" + "=" * 70)
        print("  📍 GPS & RTK STATIONARY DRIFT BENCHMARK")
        print("=" * 70)
        print(f"Collecting {self.target_samples} samples from /fix (Keep robot stationary)...\n")

    def _determine_fix_type(self, msg: NavSatFix) -> str:
        """Determines exact fix type from ROS status and position covariance."""
        status = msg.status.status
        cov_var = msg.position_covariance[0] if len(msg.position_covariance) > 0 else 100.0

        if status == NavSatStatus.STATUS_GBAS_FIX or status in [4, 5]:
            # Covariance variance < 0.005 indicates Mode 4 (RTK FIX < 2 cm)
            if cov_var < 0.005:
                return "RTK_FIX"
            else:
                return "RTK_FLOAT"
        elif status == NavSatStatus.STATUS_SBAS_FIX or status == 2:
            return "DGPS"
        elif status == NavSatStatus.STATUS_FIX or status == 1:
            return "3D_SPS"
        else:
            return "NO_FIX"

    def _get_status_display(self, fix_type: str) -> str:
        if fix_type == "RTK_FIX":
            return "🎯 RTK FIX (1–2 cm)"
        elif fix_type == "RTK_FLOAT":
            return "⏳ RTK FLOAT (~20–100 cm)"
        elif fix_type == "DGPS":
            return "📡 DGPS (SBAS)"
        elif fix_type == "3D_SPS":
            return "⚠️ STANDARD 3D FIX (NOT RTK)"
        else:
            return "❌ NO FIX"

    def fix_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        fix_type = self._determine_fix_type(msg)
        self.samples.append((msg.latitude, msg.longitude, msg.altitude, fix_type))
        count = len(self.samples)
        status_disp = self._get_status_display(fix_type)

        # Real-time progress display with exact fix label
        sys.stdout.write(f"\rProgress: [{count}/{self.target_samples} samples] ({count/self.target_samples*100:.1f}%) | Current: {status_disp}   ")
        sys.stdout.flush()

        if count >= self.target_samples:
            self.is_completed = True
            print("\n")
            self.compute_and_display_results()
            rclpy.shutdown()

    def _calculate_stats(self, sample_subset: List[Tuple[float, float, float, str]]) -> dict:
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

    def _print_stat_table(self, title: str, stats: dict) -> None:
        print("-" * 70)
        print(f"  {title} ({stats['count']} samples):")
        print("-" * 70)
        print(f"  • East-West Drift StdDev (σX)  : {stats['std_x']:.2f} cm")
        print(f"  • North-South Drift StdDev (σY): {stats['std_y']:.2f} cm")
        print(f"  • Vertical Drift StdDev (σZ)   : {stats['std_z']:.2f} cm")
        print(f"  • Average Radial Error         : {stats['avg_drift']:.2f} cm")
        print(f"  • Maximum Peak Drift           : {stats['max_drift']:.2f} cm")
        print(f"  • 50% CEP Radius (50% of time) : < {stats['cep_50']:.2f} cm")
        print(f"  • 95% 2DRMS (95% confidence)   : < {stats['drms_2d']:.2f} cm")

    def compute_and_display_results(self) -> None:
        if not self.samples:
            print("No valid GPS samples collected.")
            return

        total_time = time.time() - self.start_time
        total_count = len(self.samples)

        # Bucket samples by fix type
        rtk_fix_samples = [s for s in self.samples if s[3] == "RTK_FIX"]
        rtk_float_samples = [s for s in self.samples if s[3] == "RTK_FLOAT"]
        dgps_samples = [s for s in self.samples if s[3] == "DGPS"]
        sps_samples = [s for s in self.samples if s[3] == "3D_SPS"]

        all_stats = self._calculate_stats(self.samples)
        fix_stats = self._calculate_stats(rtk_fix_samples)
        float_stats = self._calculate_stats(rtk_float_samples)

        print("-" * 70)
        print(f"  Benchmark Duration : {total_time:.1f} seconds ({total_count} total samples)")
        print(f"  Mean Position      : {all_stats['mean_lat']:.8f}°, {all_stats['mean_lon']:.8f}° ({all_stats['mean_alt']:.2f} m)")
        print(f"  Google Maps Link   : https://www.google.com/maps?q={all_stats['mean_lat']:.8f},{all_stats['mean_lon']:.8f}")
        print("-" * 70)
        print("  🛰️ EXACT FIX QUALITY BREAKDOWN:")
        print("-" * 70)
        print(f"  • 🎯 RTK FIX (1–2 cm)          : {len(rtk_fix_samples) / total_count * 100:.1f}% ({len(rtk_fix_samples)} samples)")
        print(f"  • ⏳ RTK FLOAT (~20–100 cm)    : {len(rtk_float_samples) / total_count * 100:.1f}% ({len(rtk_float_samples)} samples)")
        print(f"  • 📡 DGPS (SBAS)               : {len(dgps_samples) / total_count * 100:.1f}% ({len(dgps_samples)} samples)")
        print(f"  • ⚠️ STANDARD 3D (NOT RTK)     : {len(sps_samples) / total_count * 100:.1f}% ({len(sps_samples)} samples)")

        # Print distinct tables
        if fix_stats:
            self._print_stat_table("🎯 TRUE RTK FIX PRECISION (SURVEY-GRADE)", fix_stats)

        if float_stats:
            self._print_stat_table("⏳ RTK FLOAT PRECISION (APPROXIMATED PHASE)", float_stats)

        self._print_stat_table("📊 OVERALL COMBINED SAMPLES", all_stats)

        print("=" * 70 + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="GPS & RTK Precision Drift Measurement")
    parser.add_argument('--samples', type=int, default=3000, help="Target samples to collect (default: 3000, press Ctrl+C anytime to stop early)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GPSDriftBenchmark(target_samples=args.samples)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.is_completed:
            print("\n\n⏹️ Benchmark stopped by user (Ctrl+C). Computing statistics for collected samples...")
            node.compute_and_display_results()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
