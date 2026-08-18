#!/usr/bin/env bash
# ==============================================================================
# Automated Raspberry Pi 5 Wi-Fi Auto-Connect & Stability Setup Script
# ==============================================================================
# Usage:
#   sudo bash documentation/scripts/setup_wifi.sh
# ==============================================================================

set -e

if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run as root: sudo bash documentation/scripts/setup_wifi.sh"
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

echo "⚙️ [1/4] Creating TP-Link USB Wi-Fi persistent udev rule..."
echo 'SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2357", ATTRS{idProduct}=="010c", NAME="wlan1"' > /etc/udev/rules.d/70-tplink-wifi.rules
udevadm control --reload-rules && udevadm trigger 2>/dev/null || true

echo "⚙️ [2/4] Disabling Wi-Fi power save mode permanently (prevents drops)..."
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
systemctl enable --now wifi-powersave-off.service 2>/dev/null || true

echo "⚙️ [3/4] Writing Netplan configuration..."
if [ "$HAS_WLAN1" = true ]; then
    echo "   -> Detected external antenna (wlan1). Configuring dual-antenna failover..."
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
        route-metric: 100
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
else
    echo "   -> Configuring built-in Wi-Fi (wlan0)..."
    cat <<EOF > /etc/netplan/50-cloud-init.yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        "${SSID}":
          password: "${PASSWORD}"
EOF
fi

chmod 600 /etc/netplan/50-cloud-init.yaml

echo "⚙️ [4/4] Applying Netplan network settings..."
netplan generate
systemctl restart systemd-networkd 2>/dev/null || true
systemctl restart "netplan-wpa-wlan0.service" 2>/dev/null || true

echo ""
echo "=================================================================="
echo " ✅ Wi-Fi Auto-Connect & Stability Setup Complete!"
echo "=================================================================="
echo "Connected IP Addresses:"
ip -4 addr show | grep -E "inet " | grep -v "127.0.0.1" || echo "Acquiring DHCP IP..."
echo "=================================================================="
