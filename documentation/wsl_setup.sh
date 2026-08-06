#!/usr/bin/env bash
# ==============================================================================
# ROS 2 WSL 2 Ubuntu Setup Script (For Windows Laptop)
# Supports Ubuntu 24.04 (Jazzy) and Ubuntu 22.04 (Humble)
# ==============================================================================

set -e

echo "=== Detecting Ubuntu Version in WSL ==="
UBUNTU_CODENAME=$(lsb_release -cs)
echo "Detected Ubuntu Codename: $UBUNTU_CODENAME"

if [ "$UBUNTU_CODENAME" = "noble" ]; then
    ROS_DISTRO="jazzy"
elif [ "$UBUNTU_CODENAME" = "jammy" ]; then
    ROS_DISTRO="humble"
else
    echo "Unsupported Ubuntu version ($UBUNTU_CODENAME). Recommended: Ubuntu 24.04 (noble) or 22.04 (jammy)."
    exit 1
fi

echo "=== Installing ROS 2 $ROS_DISTRO ==="

sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y

# Add ROS 2 GPG Key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

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
