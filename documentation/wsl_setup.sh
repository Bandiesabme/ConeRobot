#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Jazzy Installer for Ubuntu 24.04 LTS (Noble) WSL 2
# ==============================================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
RELEASE=$(lsb_release -rs 2>/dev/null || echo "unknown")

echo -e "${BLUE}=== Checking Ubuntu Version ===${NC}"
if [ "$CODENAME" != "noble" ]; then
    echo -e "${RED}Error: Unsupported Ubuntu version: Ubuntu $RELEASE ($CODENAME).${NC}"
    echo -e "${RED}This script requires Ubuntu 24.04 LTS (noble). Please use Ubuntu 24.04 in WSL.${NC}"
    exit 1
fi

echo -e "${GREEN}Ubuntu 24.04 LTS (noble) verified.${NC}"

echo -e "${BLUE}[1/3] Setting up ROS 2 Jazzy repository...${NC}"
sudo apt update -qq && sudo apt install -y -qq curl gnupg lsb-release ca-certificates
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "${BLUE}[2/3] Installing ROS 2 Jazzy base & colcon build tools...${NC}"
sudo apt update -qq
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init -q || true
fi
rosdep update -q || true

echo -e "${BLUE}[3/3] Configuring ~/.bashrc environment...${NC}"
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Setup" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

echo -e "${GREEN}====================================================${NC}"
echo -e "${GREEN} ROS 2 Jazzy installed successfully on Ubuntu 24.04!${NC}"
echo -e "${GREEN} ROS_DOMAIN_ID=42 configured in ~/.bashrc${NC}"
echo -e "${GREEN} Run: source ~/.bashrc${NC}"
echo -e "${GREEN}====================================================${NC}"
