#!/usr/bin/env python3
"""
ROS 2 Jazzy & SensorDataQoS Compatibility Patch for rf2o_laser_odometry
1. Switches LaserScan subscriber to rclcpp::SensorDataQoS() so it receives YDLidar's BestEffort scans.
2. Injects fast compilation flags into CMakeLists.txt to avoid GCC memory thrashing.
"""

import sys
import os
import re

def patch_rf2o(rf2o_dir):
    if not os.path.exists(rf2o_dir):
        print(f"Directory not found: {rf2o_dir}")
        return False

    cpp_path = os.path.join(rf2o_dir, "src", "rf2o_laser_odometry_node.cpp")
    cmake_path = os.path.join(rf2o_dir, "CMakeLists.txt")

    # 1. Patch C++ node for SensorDataQoS
    if os.path.exists(cpp_path):
        with open(cpp_path, 'r', encoding='utf-8') as f:
            code = f.read()

        # Replace any standard QoS with SensorDataQoS for the laser subscription
        patched_code = re.sub(
            r'(create_subscription<sensor_msgs::msg::LaserScan>\s*\([^,]+,\s*)(1|\d+|rclcpp::QoS\([^\)]+\))(\s*,)',
            r'\1rclcpp::SensorDataQoS()\3',
            code
        )

        if patched_code != code:
            with open(cpp_path, 'w', encoding='utf-8') as f:
                f.write(patched_code)
            print(f"Patched {cpp_path} to use rclcpp::SensorDataQoS()!")
        else:
            print(f"SensorDataQoS already set or checked in {cpp_path}.")

    # 2. Patch CMakeLists.txt for fast ARM compilation
    if os.path.exists(cmake_path):
        with open(cmake_path, 'r', encoding='utf-8') as f:
            cmake_code = f.read()

        if "fno-var-tracking" not in cmake_code:
            cmake_code = cmake_code.replace('-O3', '')
            cmake_code = "add_compile_options(-O1 -fno-var-tracking -fno-var-tracking-assignments)\n" + cmake_code
            with open(cmake_path, 'w', encoding='utf-8') as f:
                f.write(cmake_code)
            print(f"Patched {cmake_path} for fast ARM compilation!")

    return True

if __name__ == '__main__':
    target = sys.argv[1] if len(sys.argv) > 1 else "src/rf2o_laser_odometry"
    patch_rf2o(target)
