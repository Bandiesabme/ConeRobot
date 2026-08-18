#!/usr/bin/env bash
# ==============================================================================
# Automated Raspberry Pi 5 Wi-Fi Auto-Connect & Stability Setup Script
# ==============================================================================
# Usage:
#   sudo bash documentation/scripts/setup_wifi.sh
#   or with custom credentials:
#   sudo bash documentation/scripts/setup_wifi.sh "YOUR_SSID" "YOUR_PASSWORD"
# ==============================================================================

set -e

if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run as root: sudo bash documentation/scripts/setup_wifi.sh"
    exit 1
fi

# Pre-filled defaults (can be overridden via command-line arguments)
SSID="${1:-Bandi}"
PASSWORD="${2:-1234445678}"

echo "=================================================================="
echo " 📶 Configuring Wi-Fi Auto-Connect for SSID: ${SSID}"
echo "=================================================================="

echo "⚙️ [1/4] Writing Netplan configuration with dual-antenna failover..."

cat <<EOF > /etc/netplan/50-cloud-init.yaml
network:
  version: 2
  wifis:
    # High-gain external USB antenna (wlan1 - Priority 1)
    wlan1:
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 100
      access-points:
        "${SSID}":
          password: "${PASSWORD}"
    # Built-in internal antenna (wlan0 - Fallback)
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

echo "⚙️ [2/4] Applying Netplan network settings..."
netplan apply

echo "⚙️ [3/4] Creating TP-Link USB Wi-Fi persistent udev rule..."
echo 'SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2357", ATTRS{idProduct}=="010c", NAME="wlan1"' > /etc/udev/rules.d/70-tplink-wifi.rules
udevadm control --reload-rules && udevadm trigger || true

echo "⚙️ [4/4] Disabling Wi-Fi power save mode permanently (prevents drops)..."
apt-get update -qq && apt-get install -y -qq iw

cat << 'EOF' > /etc/systemd/system/wifi-powersave-off.service
[Unit]
Description=Disable Wi-Fi Power Save
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now wifi-powersave-off.service

echo ""
echo "=================================================================="
echo " ✅ Wi-Fi Auto-Connect & Stability Setup Complete!"
echo "=================================================================="
echo "Connected IP Addresses:"
ip -4 addr show | grep -E "inet " | grep -v "127.0.0.1" || echo "Connecting to Wi-Fi..."
echo "=================================================================="
