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

if [ "$DETECTED_CODENAME" = "resolute" ]; then
    echo "=== Installing Ubuntu 26.04 (Resolute) compatibility shims for ROS 2 Jazzy ==="

    sudo apt update && sudo apt install -y software-properties-common

    # ------------------------------------------------------------------
    # Shim 1: libtinyxml2-10
    # Ubuntu 26.04 ships libtinyxml2-11; ROS 2 Jazzy declares a hard APT
    # dependency on libtinyxml2-10. Fix: equivs dummy package + .so shim.
    # ------------------------------------------------------------------
    sudo apt-get install -y libtinyxml2-11 equivs

    EQUIVS_DIR=$(mktemp -d)
    cat > "$EQUIVS_DIR/libtinyxml2-10.ctl" << 'EOF'
Section: libs
Priority: optional
Standards-Version: 3.9.2
Package: libtinyxml2-10
Version: 10.0.0+dfsg-1
Provides: libtinyxml2-10
Depends: libtinyxml2-11
Architecture: amd64
Description: Dummy package providing libtinyxml2-10 via libtinyxml2-11 shim
 Ubuntu 26.04 (Resolute) ships libtinyxml2-11 which is ABI compatible
 with libtinyxml2-10. This dummy package satisfies the APT dependency
 declared by ROS 2 Jazzy packages.
EOF

    (cd "$EQUIVS_DIR" && equivs-build libtinyxml2-10.ctl)
    sudo dpkg -i "$EQUIVS_DIR"/libtinyxml2-10_*.deb
    rm -rf "$EQUIVS_DIR"

    TINYXML2_SO=$(find /usr/lib -name "libtinyxml2.so.11*" -not -name "*.deb" | sort | head -1)
    if [ -n "$TINYXML2_SO" ]; then
        echo "Creating libtinyxml2.so.10 shim -> $TINYXML2_SO"
        sudo ln -sf "$TINYXML2_SO" /usr/lib/x86_64-linux-gnu/libtinyxml2.so.10
        sudo ldconfig
    fi

    # ------------------------------------------------------------------
    # Shim 2: Python 3.12
    # Ubuntu 26.04 ships Python 3.13; ROS 2 Jazzy binaries require
    # libpython3.12t64. Install Python 3.12 from the deadsnakes PPA.
    # ------------------------------------------------------------------
    echo "=== Installing Python 3.12 via deadsnakes PPA (required by ROS 2 Jazzy) ==="
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt-get update
    sudo apt-get install -y python3.12 libpython3.12t64 python3.12-dev python3.12-distutils || \
        sudo apt-get install -y python3.12 libpython3.12t64 python3.12-dev
fi

echo "=== Installing ROS 2 $ROS_DISTRO ==="

sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y || true

# Add ROS 2 GPG Key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Repository using compatible APT suite (noble/jammy)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $APT_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-ros-base

# Install colcon and rosdep:
# On Ubuntu 26.04 the apt packages from the ROS noble repo have Python 3.12
# version conflicts, so install via pip instead (canonical method anyway).
if [ "$DETECTED_CODENAME" = "resolute" ]; then
    echo "=== Installing colcon and rosdep via pip (Ubuntu 26.04 compatibility) ==="
    sudo apt-get install -y python3-pip python3-venv
    pip3 install --break-system-packages colcon-common-extensions
    pip3 install --break-system-packages rosdep
else
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
fi

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