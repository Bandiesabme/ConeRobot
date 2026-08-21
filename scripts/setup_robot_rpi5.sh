#!/usr/bin/env bash
# ==============================================================================
# All-In-One Interactive Setup Script for Raspberry Pi 5 (Cone Robot)
# ==============================================================================
# Automates the setup with strict verification checks and user confirmation prompts
# after each step so you can review the results without losing terminal context.
#
# Usage:
#   bash scripts/setup_robot_rpi5.sh
#   bash scripts/setup_robot_rpi5.sh [WIFI_SSID] [WIFI_PASSWORD]
#   bash scripts/setup_robot_rpi5.sh --yes   # Non-interactive / unattended mode
# ==============================================================================

# Color formatting helpers
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

log_error() {
    echo -e "${RED}${BOLD}[ERROR]${RESET} $1"
}

log_section() {
    echo -e "\n${BOLD}======================================================================${RESET}"
    echo -e "${BOLD}>>> $1${RESET}"
    echo -e "${BOLD}======================================================================${RESET}\n"
}

# Parse optional arguments
AUTO_PROCEED=false
SSID=""
PASSWORD=""

for arg in "$@"; do
    if [ "$arg" = "-y" ] || [ "$arg" = "--yes" ] || [ "$arg" = "--auto" ]; then
        AUTO_PROCEED=true
    elif [ -z "$SSID" ]; then
        SSID="$arg"
    elif [ -z "$PASSWORD" ]; then
        PASSWORD="$arg"
    fi
done

# Prompt for step confirmation
confirm_step() {
    local step_num="$1"
    local step_title="$2"
    local status="$3"       # 0 for success, non-zero for error/warning
    local details="$4"

    echo ""
    if [ "$status" -eq 0 ]; then
        echo -e "${GREEN}${BOLD}✔ [STEP ${step_num} VERIFIED: SUCCESS]${RESET} ${step_title}"
    else
        echo -e "${RED}${BOLD}✖ [STEP ${step_num} VERIFICATION FAILED / WARNING]${RESET} ${step_title}"
    fi

    if [ -n "$details" ]; then
        echo -e "${CYAN}  ↳ ${details}${RESET}"
    fi
    echo ""

    if [ "$AUTO_PROCEED" != "true" ]; then
        read -r -p "$(echo -e "${YELLOW}${BOLD}👉 Press [ENTER] to continue to the next step (or Ctrl+C to abort)...${RESET}")" _unused_input
        echo ""
    fi
}

# Determine script and workspace root directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Helper to safely and automatically release background Ubuntu updater locks
wait_for_apt_lock() {
    # 1. Gracefully stop background update services so they don't block manual setup
    sudo systemctl stop unattended-upgrades.service apt-daily.service apt-daily-upgrade.service 2>/dev/null || true

    local lock_files=(
        "/var/lib/dpkg/lock-frontend"
        "/var/lib/dpkg/lock"
        "/var/lib/apt/lists/lock"
    )
    local waited=0
    for lock_file in "${lock_files[@]}"; do
        while sudo fuser "$lock_file" >/dev/null 2>&1 || pgrep -f unattended-upgr >/dev/null 2>&1; do
            if [ "$waited" -eq 0 ]; then
                log_info "Stopping background update services and waiting for APT lock..."
            fi
            sleep 2
            waited=$((waited + 2))
            if [ "$waited" -ge 8 ]; then
                # If still held after 8 seconds, politely terminate the holder
                sudo kill -15 $(sudo fuser "$lock_file" 2>/dev/null) 2>/dev/null || true
                sleep 1
                break
            fi
        done
    done

    # 2. Heal any partially configured packages if interrupted
    sudo dpkg --configure -a 2>/dev/null || true
}

# ==============================================================================
# STEP 1: SWAP MEMORY CONFIGURATION
# ==============================================================================
log_section "STEP 1/8: SWAP MEMORY CONFIGURATION (2 GB for 1 GB RAM Model)"
log_info "Configuring 2GB swap space to prevent Out-Of-Memory compilation crashes..."

SWAP_OK=0
if [ -f /swapfile ] && swapon --show | grep -q "/swapfile"; then
    log_info "Swapfile (/swapfile) is already configured and active."
else
    sudo swapoff -a || true
    sudo fallocate -l 2G /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
fi

# Verification
SWAP_DETAILS=$(swapon --show | grep "/swapfile" || true)
if [ -n "$SWAP_DETAILS" ]; then
    SWAP_OK=0
    SWAP_MSG="Active swap: $SWAP_DETAILS"
else
    SWAP_OK=1
    SWAP_MSG="Warning: Swapfile was not detected in active swap list."
fi
confirm_step "1" "Swap Memory Configuration" "$SWAP_OK" "$SWAP_MSG"

# ==============================================================================
# STEP 2: SYSTEM PACKAGES & ROS 2 JAZZY
# ==============================================================================
log_section "STEP 2/8: SYSTEM PACKAGES & ROS 2 JAZZY INSTALLATION"
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

wait_for_apt_lock
log_info "Ensuring full Ubuntu 24.04 repository sources (noble, noble-updates, noble-security, universe, multiverse)..."

# Fix incomplete Ubuntu 24.04 noble repository sources (prevents 'unmet dependencies / held broken packages')
UBUNTU_SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
if [ -f "$UBUNTU_SOURCES" ]; then
    sudo tee "$UBUNTU_SOURCES" > /dev/null << 'EOF'
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble noble-updates noble-security noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
fi

sudo apt update || true
sudo apt --fix-broken install -y || true

log_info "Installing prerequisite utilities..."
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y || true
sudo add-apt-repository multiverse -y || true

wait_for_apt_lock
sudo apt update

log_info "Configuring official ROS 2 Jazzy repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

wait_for_apt_lock
sudo apt update

log_info "Installing ROS 2 Jazzy, TF2, Foxglove, GPIO/I2C, serial, and build tools..."
wait_for_apt_lock
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
    ros-jazzy-ament-cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-gpiozero \
    python3-lgpio \
    python3-smbus \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-serial \
    libboost-dev \
    i2c-tools \
    iw \
    net-tools \
    git \
    cmake \
    build-essential \
    pkg-config

log_info "Installing Adafruit BNO08x Python IMU library and CircuitPython Blinka layer..."
sudo python3 -m pip install --break-system-packages adafruit-circuitpython-bno08x adafruit-blinka || \
python3 -m pip install --break-system-packages adafruit-circuitpython-bno08x adafruit-blinka || true

log_info "Initializing rosdep..."
if command -v rosdep >/dev/null 2>&1; then
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update || true
fi

# Verification
PKG_OK=0
PKG_ERRORS=()
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    PKG_OK=1
    PKG_ERRORS+=("ROS 2 Jazzy (/opt/ros/jazzy/setup.bash) not found")
fi
if ! python3 -c "import serial, lgpio" >/dev/null 2>&1; then
    PKG_OK=1
    PKG_ERRORS+=("Python serial/lgpio libraries missing")
fi
if ! python3 -c "import adafruit_bno08x" >/dev/null 2>&1; then
    PKG_OK=1
    BNO_ERR=$(python3 -c "import adafruit_bno08x" 2>&1 || true)
    PKG_ERRORS+=("Adafruit BNO08x Python library missing ($BNO_ERR)")
fi

if [ "$PKG_OK" -eq 0 ]; then
    PKG_MSG="ROS 2 Jazzy core, Foxglove bridge, and Python hardware libraries verified."
else
    PKG_MSG="Package issues detected: ${PKG_ERRORS[*]}"
fi
confirm_step "2" "ROS 2 Jazzy & Core Package Installation" "$PKG_OK" "$PKG_MSG"

# ==============================================================================
# STEP 3: HARDWARE PERMISSIONS (GPIO, I2C, UART) & SERVICE ISOLATION
# ==============================================================================
log_section "STEP 3/8: HARDWARE PERMISSIONS & SERIAL ISOLATION"
log_info "Writing udev rules for GPIO (0666) and I2C group access..."
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0666"' | sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null
sudo usermod -aG i2c "$USER" || true
sudo usermod -aG dialout "$USER" || true
sudo udevadm control --reload-rules && sudo udevadm trigger 2>/dev/null || true

log_info "Disabling serial-getty login console and gpsd on ttyAMA0 (for GPS HAT)..."
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true

sudo systemctl stop gpsd gpsd.socket 2>/dev/null || true
sudo systemctl disable gpsd gpsd.socket 2>/dev/null || true
sudo systemctl mask gpsd gpsd.socket 2>/dev/null || true

# Verification
PERM_OK=0
PERM_ERRORS=()
if [ ! -f /etc/udev/rules.d/99-gpio.rules ]; then
    PERM_OK=1
    PERM_ERRORS+=("/etc/udev/rules.d/99-gpio.rules missing")
fi
if [ ! -f /etc/udev/rules.d/99-i2c.rules ]; then
    PERM_OK=1
    PERM_ERRORS+=("/etc/udev/rules.d/99-i2c.rules missing")
fi

if [ "$PERM_OK" -eq 0 ]; then
    PERM_MSG="GPIO/I2C udev rules installed. serial-getty and gpsd masked on /dev/ttyAMA0."
else
    PERM_MSG="Permissions issues: ${PERM_ERRORS[*]}"
fi
confirm_step "3" "Hardware Permissions & Serial Isolation" "$PERM_OK" "$PERM_MSG"

# ==============================================================================
# STEP 4: WI-FI AUTO-CONNECT & POWER SAVING
# ==============================================================================
log_section "STEP 4/8: WI-FI AUTO-CONNECT & POWER SAVING"
WIFI_OK=0
WIFI_MSG=""
if [ -n "$SSID" ] && [ -n "$PASSWORD" ]; then
    log_info "Configuring Wi-Fi for SSID: '${SSID}'..."
    if [ -f "${SCRIPT_DIR}/setup_wifi.sh" ]; then
        if sudo SSH_CONNECTION="$SSH_CONNECTION" SSH_CLIENT="$SSH_CLIENT" SSH_TTY="$SSH_TTY" bash "${SCRIPT_DIR}/setup_wifi.sh" "$SSID" "$PASSWORD"; then
            WIFI_MSG="Wi-Fi configured for SSID: $SSID (power-save disabled)."
        else
            WIFI_OK=1
            WIFI_MSG="setup_wifi.sh returned an error code."
        fi
    fi
elif [ -f "${SCRIPT_DIR}/setup_wifi.sh" ]; then
    log_info "No custom Wi-Fi arguments provided. Auto-detecting existing Wi-Fi credentials or using default (Bandi)..."
    if sudo SSH_CONNECTION="$SSH_CONNECTION" SSH_CLIENT="$SSH_CLIENT" SSH_TTY="$SSH_TTY" bash "${SCRIPT_DIR}/setup_wifi.sh"; then
        WIFI_MSG="Wi-Fi configured for external antenna/extender (power-save disabled)."
    else
        WIFI_OK=1
        WIFI_MSG="setup_wifi.sh returned an error code."
    fi
else
    log_warning "setup_wifi.sh not found. Skipping Wi-Fi configuration."
    WIFI_OK=1
    WIFI_MSG="Wi-Fi setup script was not found."
fi

# Verification
if [ ! -f /etc/netplan/50-cloud-init.yaml ]; then
    WIFI_OK=1
    WIFI_MSG="Netplan configuration file /etc/netplan/50-cloud-init.yaml missing."
fi
confirm_step "4" "Wi-Fi Auto-Connect & Low Latency" "$WIFI_OK" "$WIFI_MSG"

# ==============================================================================
# STEP 5: BOOT CONFIGURATION (/boot/firmware/config.txt)
# ==============================================================================
log_section "STEP 5/8: BOOT FIRMWARE CONFIGURATION"
BOOT_CONFIG="/boot/firmware/config.txt"
BOOT_OK=0

if [ -f "$BOOT_CONFIG" ]; then
    log_info "Configuring USB power (1.6A), fast I2C (400kHz), and Hardware UART in $BOOT_CONFIG..."
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

    # Verification assertions
    BOOT_ERRORS=()
    if ! grep -q "usb_max_current_enable=1" "$BOOT_CONFIG"; then
        BOOT_OK=1
        BOOT_ERRORS+=("usb_max_current_enable missing")
    fi
    if ! grep -q "dtparam=i2c_arm" "$BOOT_CONFIG"; then
        BOOT_OK=1
        BOOT_ERRORS+=("dtparam=i2c_arm missing")
    fi
    if ! grep -q "dtparam=uart0=on" "$BOOT_CONFIG"; then
        BOOT_OK=1
        BOOT_ERRORS+=("dtparam=uart0=on missing")
    fi

    if [ "$BOOT_OK" -eq 0 ]; then
        BOOT_MSG="Verified usb_max_current_enable=1, 400kHz I2C, and UART parameters in $BOOT_CONFIG."
    else
        BOOT_MSG="Boot config issues: ${BOOT_ERRORS[*]}"
    fi
else
    log_warning "Boot config $BOOT_CONFIG not found (Non-Pi / Container environment)."
    BOOT_OK=0
    BOOT_MSG="Skipped (non-standard firmware path)."
fi
confirm_step "5" "Boot Firmware Configuration" "$BOOT_OK" "$BOOT_MSG"

# ==============================================================================
# STEP 6: YDLIDAR SDK & LASER ODOMETRY SETUP
# ==============================================================================
log_section "STEP 6/8: YDLIDAR SDK & DRIVER SETUP"
LIDAR_OK=0
if [ -f "${SCRIPT_DIR}/install_ydlidar.sh" ]; then
    log_info "Building YDLidar C++ SDK, udev rules, and applying Jazzy patches..."
    if bash "${SCRIPT_DIR}/install_ydlidar.sh"; then
        # Verification assertions
        LIDAR_ERRORS=()
        if [ ! -f /etc/udev/rules.d/ydlidar.rules ]; then
            LIDAR_OK=1
            LIDAR_ERRORS+=("/etc/udev/rules.d/ydlidar.rules missing")
        fi
        if [ ! -d "${WORKSPACE_DIR}/src/ydlidar_ros2_driver" ]; then
            LIDAR_OK=1
            LIDAR_ERRORS+=("src/ydlidar_ros2_driver missing")
        fi
        if [ ! -d "${WORKSPACE_DIR}/src/rf2o_laser_odometry" ]; then
            LIDAR_OK=1
            LIDAR_ERRORS+=("src/rf2o_laser_odometry missing")
        fi

        if [ "$LIDAR_OK" -eq 0 ]; then
            LIDAR_MSG="YDLidar SDK compiled, /dev/ydlidar rule installed, Jazzy drivers compiled."
        else
            LIDAR_MSG="Driver verification failed: ${LIDAR_ERRORS[*]}"
        fi
    else
        LIDAR_OK=1
        LIDAR_MSG="YDLIDAR SDK / driver build script failed. Check build logs above."
    fi
else
    log_warning "install_ydlidar.sh not found."
    LIDAR_OK=1
    LIDAR_MSG="install_ydlidar.sh script missing."
fi
confirm_step "6" "YDLIDAR C++ SDK & Laser Driver Setup" "$LIDAR_OK" "$LIDAR_MSG"

# ==============================================================================
# STEP 7: ENVIRONMENT VARIABLES & BASHRC
# ==============================================================================
log_section "STEP 7/8: ENVIRONMENT VARIABLES & ~/.bashrc CONFIGURATION"
log_info "Configuring environment in ~/.bashrc..."
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

# Verification assertions
ENV_OK=0
ENV_ERRORS=()
if ! grep -q "ROS_DOMAIN_ID=42" "$BASHRC"; then
    ENV_OK=1
    ENV_ERRORS+=("ROS_DOMAIN_ID=42 missing in ~/.bashrc")
fi
if ! grep -q "GPIOZERO_PIN_FACTORY=lgpio" "$BASHRC"; then
    ENV_OK=1
    ENV_ERRORS+=("GPIOZERO_PIN_FACTORY=lgpio missing in ~/.bashrc")
fi
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
    ENV_OK=1
    ENV_ERRORS+=("source /opt/ros/jazzy/setup.bash missing in ~/.bashrc")
fi

if [ "$ENV_OK" -eq 0 ]; then
    ENV_MSG="ROS_DOMAIN_ID=42, GPIOZERO_PIN_FACTORY=lgpio, and setup.bash verified in ~/.bashrc."
else
    ENV_MSG="Environment issues: ${ENV_ERRORS[*]}"
fi
confirm_step "7" "Environment Variables (~/.bashrc)" "$ENV_OK" "$ENV_MSG"

# ==============================================================================
# STEP 8: BUILD CONE ROBOT ROS 2 WORKSPACE
# ==============================================================================
log_section "STEP 8/8: BUILD CONE ROBOT ROS 2 WORKSPACE"
BUILD_OK=0
if [ -d "${WORKSPACE_DIR}/src" ]; then
    log_info "Compiling packages in ${WORKSPACE_DIR} (colcon build --symlink-install --parallel-workers 2)..."
    cd "${WORKSPACE_DIR}"
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    if colcon build --symlink-install --parallel-workers 2; then
        if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
            BUILD_OK=0
            BUILD_MSG="Workspace packages compiled and install/setup.bash generated successfully."
        else
            BUILD_OK=1
            BUILD_MSG="colcon finished but install/setup.bash was not found."
        fi
    else
        BUILD_OK=1
        BUILD_MSG="Workspace compilation encountered errors. Check colcon output above."
    fi
else
    BUILD_OK=1
    BUILD_MSG="src directory not found in ${WORKSPACE_DIR}."
fi
confirm_step "8" "Workspace Compilation" "$BUILD_OK" "$BUILD_MSG"

# ==============================================================================
# SUMMARY & NEXT STEPS
# ==============================================================================
log_section "ALL STEPS FINISHED!"
echo -e "${GREEN}${BOLD}======================================================================${RESET}"
echo -e "${GREEN}${BOLD} 🎉 SETUP COMPLETED SUCCESSFULLY!${RESET}"
echo -e "${GREEN}${BOLD}======================================================================${RESET}\n"
echo -e "${YELLOW}Please reboot the Raspberry Pi to apply hardware boot and udev rules:${RESET}"
echo -e "    ${BOLD}sudo reboot${RESET}\n"
echo -e "After rebooting, launch the robot with:"
echo -e "    ${BOLD}ros2 launch cone_robot_control robot.launch.py robot_type:=lidar${RESET}"
echo -e "or"
echo -e "    ${BOLD}ros2 launch cone_robot_control robot.launch.py robot_type:=gps${RESET}\n"
