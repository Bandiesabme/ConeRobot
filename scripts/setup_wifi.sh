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

SSID="${1:-}"
PASSWORD="${2:-}"

# Auto-detect existing Wi-Fi credentials from current Netplan config if not passed
EXISTING_NETPLAN="/etc/netplan/50-cloud-init.yaml"
if [ -z "$SSID" ] && [ -f "$EXISTING_NETPLAN" ]; then
    DETECTED_SSID=$(grep -E '^\s+["'\''a-zA-Z0-9_-]+":' "$EXISTING_NETPLAN" | grep -v 'wlan' | grep -v 'version' | head -n 1 | tr -d '": \t' || true)
    DETECTED_PASS=$(grep -E 'password:\s*' "$EXISTING_NETPLAN" | head -n 1 | sed 's/.*password:\s*//; s/["'\'']//g; s/\s*$//' || true)
    if [ -n "$DETECTED_SSID" ] && [ -n "$DETECTED_PASS" ]; then
        SSID="$DETECTED_SSID"
        PASSWORD="$DETECTED_PASS"
        echo "🔍 Auto-detected existing Wi-Fi configuration (SSID: '${SSID}')"
    fi
fi

# Fallback to default credentials if none provided and none detected
SSID="${SSID:-Bandi}"
PASSWORD="${PASSWORD:-1234445678}"

echo "=================================================================="
echo " 📶 Configuring Wi-Fi Auto-Connect for SSID: ${SSID}"
echo "=================================================================="

# Check if an external USB Wi-Fi adapter (wlan1) is physically plugged in
HAS_WLAN1=false
if [ -d "/sys/class/net/wlan1" ] || [ -d "/sys/class/net/wlx*" ]; then
    HAS_WLAN1=true
fi

echo "⚙️ [1/4] Setting up USB Wi-Fi persistent naming & smart hotplug rules..."
# 1. Device naming rule
cat << 'EOF' > /etc/udev/rules.d/70-wifi-naming.rules
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="rtl8xxxu|rtw88_*|mt76*", NAME="wlan1"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2357", NAME="wlan1"
EOF

# 2. Automated Smart Switching Script
cat << 'EOF' > /usr/local/bin/wifi-hotplug-handler.sh
#!/bin/bash
ACTION="$1"

if [ "$ACTION" = "add" ]; then
    # Trigger netplan so wlan1 connects to Wi-Fi
    netplan apply 2>/dev/null || true

    # In background: wait until wlan1 has an IP address, THEN turn off wlan0 to eliminate RF interference
    (
        for i in {1..25}; do
            sleep 1
            if ip -4 addr show wlan1 2>/dev/null | grep -q "inet "; then
                # wlan1 is connected with an IP -> shut down onboard wlan0 to eliminate interference
                ip link set wlan0 down 2>/dev/null || true
                break
            fi
        done
    ) &
elif [ "$ACTION" = "remove" ]; then
    # When wlan1 is unplugged, restore wlan0 immediately
    ip link set wlan0 up 2>/dev/null || true
    netplan apply 2>/dev/null || true
fi
EOF
chmod +x /usr/local/bin/wifi-hotplug-handler.sh

# 3. Udev hotplug trigger
cat << 'EOF' > /etc/udev/rules.d/99-wifi-hotplug.rules
SUBSYSTEM=="net", ACTION=="add", KERNEL=="wlan1", RUN+="/usr/local/bin/wifi-hotplug-handler.sh add"
SUBSYSTEM=="net", ACTION=="remove", KERNEL=="wlan1", RUN+="/usr/local/bin/wifi-hotplug-handler.sh remove"
EOF

udevadm control --reload-rules && udevadm trigger 2>/dev/null || true

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
