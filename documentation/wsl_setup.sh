#!/usr/bin/env bash
# ==============================================================================
# ROS 2 WSL 2 Ubuntu Setup Script (For Windows Laptop)
# Supports Ubuntu 26.04 (Resolute), 24.04 (Noble), and 22.04 (Jammy)
# ==============================================================================

set -e

echo "=== Detecting Ubuntu Version in WSL ==="
DETECTED_CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
echo "Detected Ubuntu Codename: $DETECTED_CODENAME"

if [ "$DETECTED_CODENAME" = "jammy" ]; then
    ROS_DISTRO="humble"
    APT_CODENAME="jammy"
else
    # Default for Ubuntu 24.04 (noble), 26.04 (resolute), or rolling builds
    ROS_DISTRO="jazzy"
    APT_CODENAME="noble"
    echo "Mapping Ubuntu '$DETECTED_CODENAME' to ROS 2 '$ROS_DISTRO' (APT suite: $APT_CODENAME)"
fi

echo "=== Installing ROS 2 $ROS_DISTRO ==="

sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y || true

# Add ROS 2 GPG Key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Repository using compatible APT suite (noble/jammy)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $APT_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Add ROS 2 environment & domain ID to bashrc
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Setup" >> ~/.bashrc
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

echo ""
echo "=== ROS 2 $ROS_DISTRO Installation Complete! ==="
echo "Restart your terminal or run: source ~/.bashrc"
