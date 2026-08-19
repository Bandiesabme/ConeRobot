#!/usr/bin/env bash
# ==============================================================================
# YDLIDAR T-mini Plus Automated SDK & ROS 2 Driver Setup Script
# Installs YDLidar-SDK C++ library and clones ydlidar_ros2_driver into src/
# Target Platform: Raspberry Pi 5 / Ubuntu 24.04 (ROS 2 Jazzy)
# ==============================================================================

set -e

# Capture script directory and workspace root at the absolute beginning
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=== Workspace Root detected: $WORKSPACE_ROOT ==="

echo "=== 1. Installing Prerequisites ==="
sudo apt update
sudo apt install -y cmake build-essential git python3-colcon-common-extensions ros-jazzy-tf2-ros libboost-dev

echo "=== 2. Building and Installing YDLidar-SDK C++ Library ==="
SDK_DIR="$HOME/YDLidar-SDK"

if [ ! -d "$SDK_DIR" ]; then
    echo "Cloning YDLidar-SDK repository into $SDK_DIR..."
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
else
    echo "YDLidar-SDK directory already exists at $SDK_DIR. Updating..."
    (cd "$SDK_DIR" && git pull origin master || true)
fi

echo "Building YDLidar-SDK..."
mkdir -p "$SDK_DIR/build"
(
    cd "$SDK_DIR/build"
    cmake ..
    make -j$(nproc)
    sudo make install
)

echo "=== 3. Setting up Drivers in Workspace ==="
DRIVER_DEST="$WORKSPACE_ROOT/src/ydlidar_ros2_driver"
OLD_MISPLACED_DEST="$HOME/src/ydlidar_ros2_driver"

if [ -d "$OLD_MISPLACED_DEST" ] && [ ! -d "$DRIVER_DEST" ]; then
    echo "Found existing ydlidar_ros2_driver at $OLD_MISPLACED_DEST. Moving to $DRIVER_DEST..."
    mkdir -p "$WORKSPACE_ROOT/src"
    mv "$OLD_MISPLACED_DEST" "$DRIVER_DEST"
elif [ ! -d "$DRIVER_DEST" ]; then
    echo "Cloning ydlidar_ros2_driver into $DRIVER_DEST..."
    mkdir -p "$WORKSPACE_ROOT/src"
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$DRIVER_DEST"
else
    echo "ydlidar_ros2_driver repository already exists at $DRIVER_DEST."
fi

# Setup RF2O Laser Odometry Driver
RF2O_DEST="$WORKSPACE_ROOT/src/rf2o_laser_odometry"
if [ ! -d "$RF2O_DEST" ]; then
    echo "Cloning rf2o_laser_odometry into $RF2O_DEST..."
    git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git "$RF2O_DEST"
else
    echo "rf2o_laser_odometry repository already exists at $RF2O_DEST."
fi

echo "=== Patching rf2o_laser_odometry for ROS 2 Jazzy & SensorDataQoS ==="
PATCH_RF2O_SCRIPT="$SCRIPT_DIR/patch_rf2o_jazzy.py"
if [ -f "$PATCH_RF2O_SCRIPT" ]; then
    python3 "$PATCH_RF2O_SCRIPT" "$RF2O_DEST"
fi

echo "=== Patching ydlidar_ros2_driver_node.cpp for ROS 2 Jazzy compatibility ==="
PATCH_SCRIPT="$SCRIPT_DIR/patch_ydlidar_jazzy.py"
if [ -f "$PATCH_SCRIPT" ]; then
    python3 "$PATCH_SCRIPT" "$DRIVER_DEST/src/ydlidar_ros2_driver_node.cpp"
fi

echo "=== 4. Setting up USB udev Rules ==="
UDEV_SCRIPT="$WORKSPACE_ROOT/documentation/scripts/init_ydlidar_udev.sh"
if [ -f "$UDEV_SCRIPT" ]; then
    bash "$UDEV_SCRIPT"
elif [ -f "$SCRIPT_DIR/init_ydlidar_udev.sh" ]; then
    bash "$SCRIPT_DIR/init_ydlidar_udev.sh"
fi

echo "=== 5. Building Workspace (Fast Mode) ==="
cd "$WORKSPACE_ROOT"
# Clean stale CMake cache from previous build
rm -rf "$WORKSPACE_ROOT/build/rf2o_laser_odometry"

export MAKEFLAGS="-j1"
colcon build --symlink-install --parallel-workers 1 --executor sequential

echo ""
echo "=============================================================================="
echo "✅ Setup and Build Completed Successfully!"
echo "=============================================================================="
echo "You can now run:"
echo "   source install/setup.bash"
echo "   ros2 launch cone_robot_control robot.launch.py"
echo "=============================================================================="
