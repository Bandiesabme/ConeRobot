
#Distributor ID: Ubuntu
#Description:    Ubuntu 24.04.4 LTS
#Release:        24.04
#Codename:       noble

# ==============================================================================
# 1. SYSTEM PREPARATION & UPDATES
# ==============================================================================

# Non-interactive mode to prevent SSH hangs during package restarts
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

# Update local package lists and upgrade existing packages
sudo -E apt update && sudo -E apt upgrade -y

# Install essential helper utilities for keys, certificates, and software management
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates

# Enable universe repository
sudo add-apt-repository universe -y

# Run a full distribution upgrade to update system libraries
sudo apt update
sudo apt dist-upgrade -y


# ==============================================================================
# 2. ADD ROS 2 REPOSITORIES & GPG KEY
# ==============================================================================

# Download the official ROS 2 GPG security key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to your APT package sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# ==============================================================================
# 3. INSTALL ROS 2 JAZZY & CORE TOOLS
# ==============================================================================

# Refresh package lists to index the newly added ROS 2 repository
sudo apt update

# Install ROS 2 Base, CLI tools, desktop messages, build tools, Rosdep, Foxglove bridge, and Raspberry Pi 5 GPIO/I2C tools
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
    i2c-tools

# Install BNO08x IMU Python library
pip3 install --break-system-packages adafruit-circuitpython-bno08x

# Initialize and update rosdep database
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update


# ==============================================================================
# 4. ENVIRONMENT & NETWORK CONFIGURATION
# ==============================================================================

# Automatically load ROS 2 commands on every new terminal session
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# Set matching Domain ID for laptop/Pi Wi-Fi communication
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

# Load the updated ~/.bashrc environment into the current terminal session
source ~/.bashrc


# ==============================================================================
# 5. CREATE ROS 2 WORKSPACE & VERIFY
# ==============================================================================

# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the empty workspace
colcon build

# Add the workspace overlay to your ~/.bashrc
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

# Verify ROS 2 installation
ros2 --help