#!/usr/bin/env bash
# ==============================================================================
# Automated Raspberry Pi 5 Wi-Fi Auto-Connect & Stability Setup Script
# ==============================================================================
# Description:
#   - Configures automatic Wi-Fi connection for Raspberry Pi 5.
#   - Disables Wi-Fi Power Saving permanently (stops ping spikes & SSH lag).
#   - Supports automatic USB Wi-Fi adapter hotplugging:
#       * When USB Wi-Fi (wlan1) is connected -> automatically disables onboard
#         wlan0 to eliminate RF interference and dual-interface route flapping.
#       * When USB Wi-Fi (wlan1) is unplugged -> automatically re-enables wlan0.
#
# Usage:
#   sudo bash scripts/setup_wifi.sh [SSID] [PASSWORD]
#   sudo bash scripts/setup_wifi.sh Bandi 1234445678
# ==============================================================================

set -e

if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run as root: sudo bash scripts/setup_wifi.sh"
    exit 1
fi

SSID="${1:-Bandi}"
PASSWORD="${2:-1234445678}"

echo "=================================================================="
echo " 📶 Configuring Wi-Fi Auto-Connect for SSID: ${SSID}"
echo "=================================================================="

# Check if an external USB Wi-Fi adapter (wlan1) is physically plugged in
HAS_WLAN1=false
if [ -d "/sys/class/net/wlan1" ] || [ -d "/sys/class/net/wlx*" ]; then
    HAS_WLAN1=true
fi

echo "⚙️ [1/5] Setting up USB Wi-Fi persistent udev naming & hotplug rules..."
# TP-Link and generic Realtek/MediaTek USB dongles
cat << 'EOF' > /etc/udev/rules.d/70-wifi-naming.rules
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="rtl8xxxu|rtw88_*|mt76*", NAME="wlan1"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2357", NAME="wlan1"
EOF

# Auto-hotplug: disable wlan0 when wlan1 is connected, re-enable when unplugged
cat << 'EOF' > /etc/udev/rules.d/99-wifi-hotplug.rules
ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlan1", RUN+="/usr/bin/ip link set wlan0 down"
ACTION=="remove", SUBSYSTEM=="net", KERNEL=="wlan1", RUN+="/usr/bin/ip link set wlan0 up"
EOF

udevadm control --reload-rules && udevadm trigger 2>/dev/null || true

echo "⚙️ [2/5] Creating NetworkManager Dispatcher auto-switch script..."
mkdir -p /etc/NetworkManager/dispatcher.d/
cat << 'EOF' > /etc/NetworkManager/dispatcher.d/99-wifi-auto-switch.sh
#!/bin/bash
INTERFACE="$1"
ACTION="$2"

if [ "$INTERFACE" = "wlan1" ]; then
    if [ "$ACTION" = "up" ]; then
        # External antenna active -> turn off internal Wi-Fi to stop dual-radio conflict
        ip link set wlan0 down 2>/dev/null || true
    elif [ "$ACTION" = "down" ]; then
        # External antenna removed -> restore internal Wi-Fi fallback
        ip link set wlan0 up 2>/dev/null || true
    fi
fi
EOF
chmod +x /etc/NetworkManager/dispatcher.d/99-wifi-auto-switch.sh

echo "⚙️ [3/5] Disabling Wi-Fi power save mode permanently (prevents SSH lag & drops)..."
if ! command -v iw >/dev/null 2>&1; then
    apt-get update -qq && apt-get install -y -qq iw
fi

# NetworkManager power save override (2 = disable powersave)
mkdir -p /etc/NetworkManager/conf.d/
cat << 'EOF' > /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
[connection]
wifi.powersave = 2
EOF

# Immediate runtime power save disable
iw dev wlan0 set power_save off 2>/dev/null || true
if [ "$HAS_WLAN1" = true ]; then
    iw dev wlan1 set power_save off 2>/dev/null || true
fi

echo "⚙️ [4/5] Writing Netplan configuration..."
cat <<EOF > /etc/netplan/50-cloud-init.yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
  wifis:
    wlan1:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 50
      access-points:
        "${SSID}":
          password: "${PASSWORD}"
    wlan0:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 600
      access-points:
        "${SSID}":
          password: "${PASSWORD}"
EOF

chmod 600 /etc/netplan/50-cloud-init.yaml

echo "⚙️ [5/5] Generating Netplan & configuring interface state (SSH-Safe)..."
netplan generate 2>/dev/null || true

# Check if currently connected via SSH to avoid killing the active session
IS_SSH=false
if [ -n "$SSH_CONNECTION" ] || [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ] || who am i 2>/dev/null | grep -q "(.*)" || pstree -s $$ 2>/dev/null | grep -q sshd; then
    IS_SSH=true
fi

if [ "$IS_SSH" = true ]; then
    echo "   -> Active SSH session detected. Network configurations written safely."
    echo "   -> Active wireless link preserved so SSH is not disconnected."
    echo "   -> New Netplan configuration will cleanly take full effect upon reboot (sudo reboot)."
else
    # If on local console or non-SSH, apply immediately
    if [ "$HAS_WLAN1" = true ]; then
        echo "   -> External antenna (wlan1) active. Disabling onboard wlan0..."
        ip link set wlan0 down 2>/dev/null || true
        ip link set wlan1 up 2>/dev/null || true
    fi
    systemctl restart NetworkManager 2>/dev/null || true
fi

echo ""
echo "=================================================================="
echo " ✅ Wi-Fi Auto-Connect & Auto-Switching Setup Complete!"
echo "=================================================================="
echo "Active Wireless Link Status:"
if [ "$HAS_WLAN1" = true ]; then
    iw dev wlan1 link 2>/dev/null || true
else
    iw dev wlan0 link 2>/dev/null || true
fi
echo "=================================================================="
