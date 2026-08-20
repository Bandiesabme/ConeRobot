#!/usr/bin/env bash
# ==============================================================================
# YDLIDAR T-mini Plus udev Rule Setup Script for Raspberry Pi 5
# Creates a permanent symlink /dev/ydlidar -> /dev/ttyUSB* (CP2102 USB converter)
# ==============================================================================

set -e

echo "=== Creating YDLIDAR T-mini Plus udev rules ==="

# Rule matches Silicon Labs CP210x USB-to-UART bridge (Vendor ID: 10c4, Product ID: ea60)
sudo tee /etc/udev/rules.d/ydlidar.rules > /dev/null << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
KERNEL=="ttyACM*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
EOF

echo "=== Reloading udev rules ==="
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

echo ""
echo "SUCCESS: udev rule created at /etc/udev/rules.d/ydlidar.rules"
echo "Unplug and re-plug your YDLIDAR USB cable, then verify with: ls -l /dev/ydlidar"
