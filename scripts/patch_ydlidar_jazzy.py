#!/usr/bin/env bash
#!/usr/bin/env python3
"""
ROS 2 Jazzy Typed Parameter Patch for ydlidar_ros2_driver
Applies correctly typed default values (int, double, bool, string) to all
node->declare_parameter(...) calls in C++ source file to prevent InvalidParameterTypeException.
"""

import sys
import os

def patch_file(cpp_path):
    if not os.path.exists(cpp_path):
        print(f"File not found: {cpp_path}")
        return False

    with open(cpp_path, 'r', encoding='utf-8') as f:
        code = f.read()

    # Exact typed parameter declarations matching C++ variable types
    replacements = {
        'node->declare_parameter("port");': 'node->declare_parameter("port", std::string("/dev/ydlidar"));',
        'node->declare_parameter("ignore_array");': 'node->declare_parameter("ignore_array", std::string(""));',
        'node->declare_parameter("frame_id");': 'node->declare_parameter("frame_id", std::string("laser_frame"));',
        'node->declare_parameter("baudrate");': 'node->declare_parameter("baudrate", 230400);',
        'node->declare_parameter("lidar_type");': 'node->declare_parameter("lidar_type", 1);',
        'node->declare_parameter("device_type");': 'node->declare_parameter("device_type", 0);',
        'node->declare_parameter("sample_rate");': 'node->declare_parameter("sample_rate", 9);',
        'node->declare_parameter("abnormal_check_count");': 'node->declare_parameter("abnormal_check_count", 4);',
        'node->declare_parameter("intensity_bit");': 'node->declare_parameter("intensity_bit", 10);',
        'node->declare_parameter("m1_mode");': 'node->declare_parameter("m1_mode", 0);',
        'node->declare_parameter("m2_mode");': 'node->declare_parameter("m2_mode", 0);',
        'node->declare_parameter("m3_mode");': 'node->declare_parameter("m3_mode", 0);',
        'node->declare_parameter("resolution_fixed");': 'node->declare_parameter("resolution_fixed", true);',
        'node->declare_parameter("fixed_resolution");': 'node->declare_parameter("fixed_resolution", true);',
        'node->declare_parameter("auto_reconnect");': 'node->declare_parameter("auto_reconnect", true);',
        'node->declare_parameter("reversion");': 'node->declare_parameter("reversion", true);',
        'node->declare_parameter("inverted");': 'node->declare_parameter("inverted", true);',
        'node->declare_parameter("isSingleChannel");': 'node->declare_parameter("isSingleChannel", false);',
        'node->declare_parameter("intensity");': 'node->declare_parameter("intensity", true);',
        'node->declare_parameter("support_motor_dtr");': 'node->declare_parameter("support_motor_dtr", false);',
        'node->declare_parameter("invalid_range_is_inf");': 'node->declare_parameter("invalid_range_is_inf", false);',
        'node->declare_parameter("debug");': 'node->declare_parameter("debug", false);',
        'node->declare_parameter("angle_min");': 'node->declare_parameter("angle_min", -180.0);',
        'node->declare_parameter("angle_max");': 'node->declare_parameter("angle_max", 180.0);',
        'node->declare_parameter("range_min");': 'node->declare_parameter("range_min", 0.01);',
        'node->declare_parameter("range_max");': 'node->declare_parameter("range_max", 64.0);',
        'node->declare_parameter("frequency");': 'node->declare_parameter("frequency", 10.0);',
    }

    # First clean any previous incorrect rclcpp::ParameterValue("") string patches
    bad_string_pattern = r'node->declare_parameter\("([a-zA-Z0-9_]+)", rclcpp::ParameterValue\(""\)\);'
    import re
    code = re.sub(bad_string_pattern, r'node->declare_parameter("\1");', code)

    # Apply exact typed parameter declarations
    for old, new in replacements.items():
        code = code.replace(old, new)

    with open(cpp_path, 'w', encoding='utf-8') as f:
        f.write(code)

    print(f"Successfully applied typed ROS 2 Jazzy parameters to {cpp_path}!")
    return True

if __name__ == '__main__':
    if len(sys.argv) > 1:
        target = sys.argv[1]
    else:
        target = "src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp"
    patch_file(target)
