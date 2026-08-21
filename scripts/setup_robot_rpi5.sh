#!/usr/bin/env bash
# ==============================================================================
# All-In-One Setup Script for Raspberry Pi 5 (1 GB RAM Edition) - Cone Robot
# ==============================================================================
# This script automates the complete initial configuration of a Raspberry Pi 5
# running Ubuntu 24.04 LTS (Noble) for ROS 2 Jazzy, hardware interfaces (GPIO,
# I2C, UART), LiDAR drivers, swap space, Wi-Fi auto-connect, and boot configuration.
#
# Usage:
#   bash scripts/setup_robot_rpi5.sh
#   bash scripts/setup_robot_rpi5.sh [WIFI_SSID] [WIFI_PASSWORD]
#   bash scripts/setup_robot_rpi5.sh "MyNetwork" "SecretPass123"
# ==============================================================================

set -e  # Exit immediately if a command exits with a non-zero status

# Color formatting helpers for clear terminal output
BOLD="\033[1m"
GREEN="\033[0;32m"
YELLOW="\033[0;33m"
CYAN="\033[0;36m"
RED="\033[0;31m"
RESET="\033[0m"

log_info() {
    echo -e "${CYAN}${BOLD}[INFO]${RESET} $1"
}

log_success() {
    echo -e "${GREEN}${BOLD}[SUCCESS]${RESET} $1"
}

log_warning() {
    echo -e "${YELLOW}${BOLD}[WARNING]${RESET} $1"
}

log_section() {
    echo -e "\n${BOLD}======================================================================${RESET}"
    echo -e "${BOLD}>>> $1${RESET}"
    echo -e "${BOLD}======================================================================${RESET}\n"
}

# Determine script and workspace root directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Optional Wi-Fi credentials (can be passed as arguments: bash setup_robot_rpi5.sh [SSID] [PASSWORD])
SSID="${1:-}"
PASSWORD="${2:-}"

log_section "STEP 1: SWAP MEMORY CONFIGURATION (2 GB for 1 GB RAM Model)"
# Why: Compiling C++ packages (YDLIDAR, RF2O laser odometry) and running multiple
# ROS nodes can spike memory usage. 2GB swap prevents Linux Out-Of-Memory (OOM) killer crashes.
if [ -f /swapfile ] && swapon --show | grep -q "/swapfile"; then
    log_info "Swapfile (/swapfile) already active. Skipping creation."
else
    log_info "Creating and enabling a 2 GB swapfile..."
    sudo swapoff -a || true
    sudo fallocate -l 2G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    log_success "2 GB swap space configured successfully."
fi

log_section "STEP 2: SYSTEM PACKAGES & ROS 2 JAZZY INSTALLATION"
# Why: Installs Ubuntu packages, official ROS 2 Jazzy core packages, Foxglove WebSockets,
# lgpio (Pi 5 GPIO backend), i2c-tools, and BNO08x Python drivers.
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

log_info "Updating package lists and adding universe repository..."
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y
sudo apt update

log_info "Configuring ROS 2 Jazzy APT repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

log_info "Installing ROS 2 Jazzy Base, CLI tools, TF2, Foxglove Bridge, and GPIO/I2C libraries..."
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-ros2cli \
    ros-jazzy-ros2launch \
    ros-jazzy-ros2topic \
    ros-jazzy-ros2node \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-gpiozero \
    python3-lgpio \
    python3-smbus \
    python3-pip \
    python3-serial \
    libboost-dev \
    i2c-tools \
    iw \
    net-tools \
    git \
    cmake \
    build-essential \
    pkg-config

log_info "Installing Adafruit BNO08x IMU Python library..."
pip3 install --break-system-packages adafruit-circuitpython-bno08x

log_info "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

log_section "STEP 3: HARDWARE PERMISSIONS (GPIO, I2C, UART) & DISABLE CONFLICTING SERVICES"
# Why:
# - Non-root users need access to /dev/gpiochip*, /dev/i2c-*, and /dev/ttyAMA0 (dialout group).
# - /dev/ttyAMA0 is normally claimed by the Linux serial login console (serial-getty).
# - gpsd daemon (if installed) locks /dev/ttyAMA0 and prevents ROS 2 LC29H node from reading NMEA sentences.

log_info "Configuring udev rules for GPIO and I2C..."
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0666"' | sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null
sudo usermod -aG i2c "$USER" || true
sudo usermod -aG dialout "$USER" || true
sudo udevadm control --reload-rules && sudo udevadm trigger

log_info "Disabling serial login getty and gpsd on ttyAMA0 for GPS HAT..."
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true

sudo systemctl stop gpsd gpsd.socket 2>/dev/null || true
sudo systemctl disable gpsd gpsd.socket 2>/dev/null || true
sudo systemctl mask gpsd gpsd.socket 2>/dev/null || true
log_success "Hardware permissions and serial port isolation applied."

log_section "STEP 4: WI-FI AUTO-CONNECT & POWER SAVING CONFIGURATION"
# Why:
# - Disables Wi-Fi power saving (prevents high latency, ping jitter, and dropped SSH sessions).
# - Sets up dual-antenna hotplugging (external high-gain antenna wlan1 priority + internal wlan0 fallback).
if [ -n "$SSID" ] && [ -n "$PASSWORD" ]; then
    log_info "Configuring Wi-Fi for SSID: '${SSID}'..."
    if [ -f "${SCRIPT_DIR}/setup_wifi.sh" ]; then
        sudo bash "${SCRIPT_DIR}/setup_wifi.sh" "$SSID" "$PASSWORD"
    fi
elif [ -f "${SCRIPT_DIR}/setup_wifi.sh" ]; then
    log_info "No Wi-Fi credentials passed in arguments. Configuring default or existing Wi-Fi settings..."
    sudo bash "${SCRIPT_DIR}/setup_wifi.sh" "Bandi" "1234445678"
else
    log_info "Skipping Wi-Fi setup script (not found)."
fi

log_section "STEP 5: RASPBERRY PI BOOT CONFIGURATION (/boot/firmware/config.txt)"
# Why:
# 1. usb_max_current_enable=1 -> Allows full 1.6A USB draw (critical for LiDAR).
# 2. dtparam=i2c_arm=on,i2c_arm_baudrate=400000 -> Fast 400kHz I2C for BNO08x IMU (50Hz telemetry).
# 3. enable_uart=1 & dtparam=uart0=on -> Hardware UART on GPIO 14/15 for LC29H GPS/RTK HAT.
BOOT_CONFIG="/boot/firmware/config.txt"
if [ -f "$BOOT_CONFIG" ]; then
    log_info "Checking $BOOT_CONFIG for USB power, I2C speed, and UART settings..."
    if ! grep -q "usb_max_current_enable=1" "$BOOT_CONFIG"; then
        echo "" | sudo tee -a "$BOOT_CONFIG"
        echo "# Enable full USB power for LiDAR and sensors" | sudo tee -a "$BOOT_CONFIG"
        echo "usb_max_current_enable=1" | sudo tee -a "$BOOT_CONFIG"
    fi

    if ! grep -q "dtparam=i2c_arm" "$BOOT_CONFIG"; then
        echo "# Enable fast 400kHz I2C bus for BNO08x IMU" | sudo tee -a "$BOOT_CONFIG"
        echo "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" | sudo tee -a "$BOOT_CONFIG"
    fi

    if ! grep -q "dtparam=uart0=on" "$BOOT_CONFIG"; then
        echo "# Enable Hardware UART for LC29H GPS HAT" | sudo tee -a "$BOOT_CONFIG"
        echo "enable_uart=1" | sudo tee -a "$BOOT_CONFIG"
        echo "dtparam=uart0=on" | sudo tee -a "$BOOT_CONFIG"
    fi
    log_success "Boot config updated."
else
    log_warning "Boot config $BOOT_CONFIG not found. If running inside a container/WSL, skip this step."
fi

log_section "STEP 6: YDLIDAR SDK & DRIVER SETUP"
# Why: Builds YDLIDAR C++ SDK, udev alias (/dev/ydlidar), and applies Jazzy compatibility patches.
if [ -f "${SCRIPT_DIR}/install_ydlidar.sh" ]; then
    log_info "Executing install_ydlidar.sh..."
    bash "${SCRIPT_DIR}/install_ydlidar.sh"
else
    log_warning "install_ydlidar.sh not found in scripts directory. Skipping LiDAR SDK build."
fi

log_section "STEP 7: ENVIRONMENT VARIABLES & BASHRC CONFIGURATION"
# Why:
# - Loads ROS 2 Jazzy on terminal startup.
# - Sets ROS_DOMAIN_ID=42 so all nodes communicate with laptop/remote station.
# - Sets GPIOZERO_PIN_FACTORY=lgpio for Pi 5 GPIO driver compatibility.
# - Overlays the ConeRobot workspace environment.
log_info "Configuring ~/.bashrc..."
BASHRC="$HOME/.bashrc"

add_to_bashrc_if_missing() {
    local line="$1"
    if ! grep -Fxq "$line" "$BASHRC"; then
        echo "$line" >> "$BASHRC"
    fi
}

add_to_bashrc_if_missing "# --- ROS 2 & ConeRobot Configuration ---"
add_to_bashrc_if_missing "source /opt/ros/jazzy/setup.bash"
add_to_bashrc_if_missing "export ROS_DOMAIN_ID=42"
add_to_bashrc_if_missing "export GPIOZERO_PIN_FACTORY=lgpio"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    add_to_bashrc_if_missing "source ${WORKSPACE_DIR}/install/setup.bash"
fi

log_success "Environment configuration written to ~/.bashrc."

log_section "STEP 8: BUILD CONE ROBOT ROS 2 WORKSPACE"
# Why: Compiles the local ROS 2 packages using 2 workers to avoid running out of memory.
if [ -d "${WORKSPACE_DIR}/src" ]; then
    log_info "Building workspace at ${WORKSPACE_DIR} (using --parallel-workers 2)..."
    cd "${WORKSPACE_DIR}"
    # Source ROS 2 base environment for this build step
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --parallel-workers 2
    # shellcheck disable=SC1091
    source install/setup.bash
    log_success "ROS 2 workspace built successfully."
fi

log_section "SETUP COMPLETE!"
echo -e "${GREEN}${BOLD}Your Raspberry Pi 5 robot setup is fully finished!${RESET}"
echo -e "${YELLOW}Please reboot the Raspberry Pi to apply kernel boot and udev changes:${RESET}"
echo -e "    ${BOLD}sudo reboot${RESET}"
echo -e "\nAfter rebooting, launch the robot with:"
echo -e "    ${BOLD}ros2 launch cone_robot_control robot.launch.py robot_type:=lidar${RESET}"
echo -e "or"
echo -e "    ${BOLD}ros2 launch cone_robot_control robot.launch.py robot_type:=gps${RESET}\n"
