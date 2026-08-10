#!/usr/bin/env python3
"""
ROS 2 Jazzy Parameter Patch for ydlidar_ros2_driver
Applies default values to all node->declare_parameter(...) calls in C++ source file.
"""

import sys
import os
import re

def patch_file(cpp_path):
    if not os.path.exists(cpp_path):
        print(f"File not found: {cpp_path}")
        return False

    with open(cpp_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Regex matches node->declare_parameter("param_name"); without default values
    pattern = r'node->declare_parameter\s*\(\s*"([^"]+)"\s*\);'
    replacement = r'node->declare_parameter("\1", rclcpp::ParameterValue(""));'

    new_content = re.sub(pattern, replacement, content)

    # Also clean up any stray backslash artifacts if present
    new_content = new_content.replace(r'\"', '"')

    with open(cpp_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"Successfully patched {cpp_path} for ROS 2 Jazzy compatibility!")
    return True

if __name__ == '__main__':
    if len(sys.argv) > 1:
        target = sys.argv[1]
    else:
        target = "src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp"
    patch_file(target)
