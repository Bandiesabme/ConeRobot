#!/usr/bin/env bash
# ==============================================================================
# Automated Raspberry Pi 5 Wi-Fi Failover & Stability Setup Script (Idempotent)
# ==============================================================================
# Description:
#   - Safe to run multiple times (idempotent): preserves existing saved Wi-Fi
#     networks, atomically validates Netplan YAML with rollback on error, and
#     cleans up legacy switcher units automatically.
#   - Designed for Ubuntu 24.04 Server (Netplan + systemd-networkd), with
#     automatic NetworkManager compatibility if active.
#   - Universal USB Wi-Fi dongle support (tested for TP-Link TL-WN722N v2/v3
#     RTL8188EU and any Linux cfg80211 USB Wi-Fi adapter):
#       * Pins onboard Pi 5 Wi-Fi (brcmfmac) -> wlan0
#       * Pins ANY USB Wi-Fi dongle          -> wlusb0 (no wlan0/wlan1 collision)
#       * Disables Wi-Fi power save permanently (both interfaces)
#       * Disables USB autosuspend on the Wi-Fi dongle (prevents random drops)
#       * Prevents same-subnet ARP flux during handover
#   - Continuous lightweight failover watchdog (wifi-failover.service):
#       * USB plugged in & stably online (carrier + IPv4 for 3s) -> cleanly
#         stops onboard wlan0 (eliminates 2.4 GHz interference & dual-IP DDS).
#       * USB unplugged -> wakes instantly via udev SIGUSR1 and restores wlan0.
#       * USB hangs / loses link / AP is 5 GHz-only (>18s offline) -> fails
#         back automatically to onboard wlan0 so the rover never goes dark.
#
# Usage:
#   sudo bash scripts/setup_wifi.sh                        # Keep existing SSIDs (or default Bandi)
#   sudo bash scripts/setup_wifi.sh "MySSID" "MyPassword"  # Add/update SSID (preserves others)
#   sudo bash scripts/setup_wifi.sh --replace "SSID" "PW"  # Replace all SSIDs with only this one
# ==============================================================================

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run as root: sudo bash scripts/setup_wifi.sh [SSID] [PASSWORD]"
    exit 1
fi

REPLACE_ALL=false
if [ "${1:-}" = "--replace" ]; then
    REPLACE_ALL=true
    shift
fi

if [ "$#" -eq 1 ]; then
    echo "❌ Password missing for SSID '$1'."
    echo "Usage: sudo bash scripts/setup_wifi.sh [--replace] [SSID] [PASSWORD]"
    exit 1
fi

REQ_SSID="${1:-}"
REQ_PASS="${2:-}"
NETPLAN_FILE="/etc/netplan/50-cloud-init.yaml"

detect_usb_wifi_iface() {
    local net_path iface
    for net_path in /sys/class/net/*; do
        [ -d "$net_path/wireless" ] || continue
        iface=$(basename "$net_path")
        if readlink -f "$net_path/device" 2>/dev/null | grep -q "/usb"; then
            echo "$iface"
            return 0
        fi
    done
    return 1
}

echo "=================================================================="
echo " 📶 ConeRobot Wi-Fi Failover & Stability Setup (Multi-Run Safe)"
echo "=================================================================="

# ------------------------------------------------------------------------------
# Step 1/6: Verify prerequisites & TL-WN722N v2 firmware/driver
# ------------------------------------------------------------------------------
echo "⚙️ [1/6] Checking tools & USB Wi-Fi firmware..."
MISSING_PKGS=()
command -v iw >/dev/null 2>&1 || MISSING_PKGS+=(iw)
command -v wpa_supplicant >/dev/null 2>&1 || MISSING_PKGS+=(wpasupplicant)
python3 -c "import yaml" >/dev/null 2>&1 || MISSING_PKGS+=(python3-yaml)
if [ "${#MISSING_PKGS[@]}" -gt 0 ]; then
    echo "   -> Installing missing packages: ${MISSING_PKGS[*]}..."
    apt-get update -qq && apt-get install -y -qq "${MISSING_PKGS[@]}"
fi

# Check Realtek RTL8188EU firmware used by TP-Link TL-WN722N v2/v3 (rtl8xxxu)
if ls /lib/firmware/rtlwifi/rtl8188eufw.bin* /usr/lib/firmware/rtlwifi/rtl8188eufw.bin* >/dev/null 2>&1; then
    echo "   ✓ RTL8188EU firmware found (TP-Link TL-WN722N v2/v3 ready)."
else
    echo "   ⚠️ RTL8188EU firmware not found. If using TL-WN722N v2/v3, run:"
    echo "      sudo apt-get update && sudo apt-get install -y linux-firmware"
fi

if command -v lsusb >/dev/null 2>&1 && lsusb 2>/dev/null | grep -qi "2357:010c"; then
    echo "   ✓ Detected TP-Link TL-WN722N v2 (2357:010c) on USB."
    echo "   ℹ️ Note: TL-WN722N v2 is 2.4 GHz only; plug into a black USB 2.0 port to protect GPS C/N0."
fi

# ------------------------------------------------------------------------------
# Step 2/6: Clean up legacy switcher files from earlier script versions
# ------------------------------------------------------------------------------
echo "⚙️ [2/6] Cleaning up any legacy Wi-Fi switcher units..."
systemctl stop 'wifi-dongle-switch@*.service' >/dev/null 2>&1 || true
systemctl disable 'wifi-dongle-switch@*.service' >/dev/null 2>&1 || true
rm -f /etc/systemd/system/wifi-dongle-switch@.service \
      /usr/local/bin/wifi-dongle-switcher.sh \
      /etc/udev/rules.d/70-wifi-naming.rules \
      /etc/udev/rules.d/99-wifi-dongle.rules

# ------------------------------------------------------------------------------
# Step 3/6: Install systemd.link naming, udev USB power & anti-ARP-flux rules
# ------------------------------------------------------------------------------
echo "⚙️ [3/6] Installing systemd.link naming & USB power rules..."
mkdir -p /usr/local/sbin /etc/systemd/network /etc/udev/rules.d /etc/sysctl.d /etc/systemd/system
SYSTEMCTL_BIN="$(command -v systemctl)"

# Override Ubuntu's /usr/lib/systemd/network/73-usb-net-by-mac.link (which names USB
# Wi-Fi dongles wlx<mac>). Priority 10 < 73 ensures ANY USB Wi-Fi dongle becomes wlusb0,
# and onboard Pi 5 Wi-Fi (brcmfmac) stays wlan0 — with zero kernel wlan0/wlan1 collision.
cat <<'EOF' > /etc/systemd/network/10-wifi-usb-dongle.link
[Match]
Type=wlan
Path=*-usb-*

[Link]
Name=wlusb0
EOF

cat <<'EOF' > /etc/systemd/network/11-wifi-onboard.link
[Match]
Type=wlan
Driver=brcmfmac

[Link]
Name=wlan0
EOF

cat <<'EOF' > /etc/udev/rules.d/70-wifi-usb-dongle.rules
# Disable USB autosuspend for TP-Link TL-WN722N v2 (2357:010c) & v3 (2357:0109)
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2357", ATTR{idProduct}=="010c", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2357", ATTR{idProduct}=="0109", TEST=="power/control", ATTR{power/control}="on"
EOF

cat <<EOF > /etc/udev/rules.d/99-wifi-failover.rules
# Wake wifi-failover watchdog immediately when wlusb0 is added, renamed (move), or removed
SUBSYSTEM=="net", ACTION=="add|move|remove", KERNEL=="wlusb0", RUN+="${SYSTEMCTL_BIN} kill --kill-who=main -s USR1 wifi-failover.service"
SUBSYSTEM=="net", ACTION=="add", ENV{ID_NET_NAME}=="wlusb0", RUN+="${SYSTEMCTL_BIN} kill --kill-who=main -s USR1 wifi-failover.service"
EOF

# Prevent same-subnet ARP flux during brief dual-interface handover
cat <<'EOF' > /etc/sysctl.d/99-wifi-failover.conf
# Prevent ARP flux when wlusb0 and wlan0 temporarily share the same subnet
net.ipv4.conf.all.arp_ignore = 1
net.ipv4.conf.all.arp_announce = 2
EOF
sysctl -p /etc/sysctl.d/99-wifi-failover.conf >/dev/null 2>&1 || true

# If NetworkManager is installed, keep its power-save override disabled too
if [ -d /etc/NetworkManager ]; then
    mkdir -p /etc/NetworkManager/conf.d
    cat <<'EOF' > /etc/NetworkManager/conf.d/default-wifi-powersave-off.conf
[connection]
wifi.powersave = 2
EOF
fi

# Reload BOTH udev rules and systemd.link files in systemd-udevd
udevadm control --reload >/dev/null 2>&1 || true

# ------------------------------------------------------------------------------
# Step 4/6: Install continuous Wi-Fi failover & power-save watchdog daemon
# ------------------------------------------------------------------------------
echo "⚙️ [4/6] Installing wifi-failover.service watchdog daemon..."

cat <<'EOF' > /usr/local/sbin/wifi-failover-watchdog.sh
#!/usr/bin/env bash
# ==============================================================================
# Continuous USB Wi-Fi Failover & Power-Save Watchdog (wlusb0 <-> wlan0)
# ==============================================================================
# - Prefers external USB Wi-Fi (wlusb0) whenever stably online (carrier + IPv4).
# - Cleanly stops onboard wlan0 once wlusb0 is stably online (3s) to eliminate
#   2.4 GHz co-channel interference, ARP flux, and dual-interface DDS multicast.
# - Immediately restores onboard wlan0 on USB unplug (SIGUSR1 wake-up).
# - Automatically fails back to onboard wlan0 if wlusb0 stays offline >= 18s
#   (e.g., dongle freeze, 5 GHz-only AP, or range drop), and hands back to
#   wlusb0 if/when it recovers.
# - Keeps Wi-Fi power save OFF and USB autosuspend OFF across reconnects/reboots.
# ==============================================================================

set -u

USB_IF="wlusb0"
INT_IF="wlan0"
POLL_SEC=3
STABLE_HANDOVER_SEC=3
FAILBACK_SEC=18
RECONFIG_RETRY_SEC=30

SLEEP_PID=""

log() {
    echo "[wifi-failover] $*"
}

wake_up() {
    if [ -n "$SLEEP_PID" ]; then
        kill "$SLEEP_PID" 2>/dev/null || true
    fi
}

cleanup() {
    wake_up
    exit 0
}

trap wake_up USR1
trap cleanup TERM INT

is_nm_active() {
    systemctl is-active --quiet NetworkManager 2>/dev/null && \
    ! systemctl is-active --quiet systemd-networkd 2>/dev/null
}

iface_exists() {
    [ -d "/sys/class/net/$1" ]
}

iface_ipv4() {
    ip -4 -o addr show dev "$1" scope global 2>/dev/null \
        | awk '{print $4}' | grep -v '^169\.254\.' | head -n 1 || true
}

iface_is_online() {
    local iface="$1"
    [ -d "/sys/class/net/$iface" ] || return 1
    local carrier
    carrier=$(cat "/sys/class/net/$iface/carrier" 2>/dev/null || echo 0)
    [ "$carrier" = "1" ] || return 1
    local ip4
    ip4=$(iface_ipv4 "$iface")
    [ -n "$ip4" ]
}

iface_is_up_or_wpa_active() {
    local iface="$1"
    [ -d "/sys/class/net/$iface" ] || return 1
    if ip link show dev "$iface" 2>/dev/null | grep -q "[,<]UP[,>]"; then
        return 0
    fi
    if ! is_nm_active && systemctl is-active --quiet "netplan-wpa-${iface}.service" 2>/dev/null; then
        return 0
    fi
    return 1
}

disable_powersave() {
    local iface="$1"
    [ -d "/sys/class/net/$iface" ] || return 0
    if command -v iw >/dev/null 2>&1; then
        local ps
        ps=$(iw dev "$iface" get power_save 2>/dev/null || true)
        if echo "$ps" | grep -qi "on"; then
            iw dev "$iface" set power_save off 2>/dev/null || true
        fi
    fi
    if readlink -f "/sys/class/net/$iface/device" 2>/dev/null | grep -q "/usb"; then
        local usb_pwr="/sys/class/net/$iface/device/../power/control"
        if [ -w "$usb_pwr" ]; then
            local cur
            cur=$(cat "$usb_pwr" 2>/dev/null || true)
            [ "$cur" = "on" ] || echo "on" > "$usb_pwr" 2>/dev/null || true
        fi
    fi
}

bring_up_iface() {
    local iface="$1"
    local do_reconfig="${2:-false}"
    [ -d "/sys/class/net/$iface" ] || return 1

    if ! ip link show dev "$iface" 2>/dev/null | grep -q "[,<]UP[,>]"; then
        ip link set dev "$iface" up 2>/dev/null || true
    fi

    if is_nm_active; then
        nmcli device set "$iface" managed yes >/dev/null 2>&1 || true
        if [ "$do_reconfig" = true ]; then
            nmcli device connect "$iface" >/dev/null 2>&1 || true
        fi
    else
        local wpa_unit="netplan-wpa-${iface}.service"
        if systemctl cat "$wpa_unit" >/dev/null 2>&1; then
            if ! systemctl is-active --quiet "$wpa_unit" 2>/dev/null; then
                systemctl reset-failed "$wpa_unit" >/dev/null 2>&1 || true
                systemctl start "$wpa_unit" >/dev/null 2>&1 || true
            fi
        fi
        if [ "$do_reconfig" = true ] && command -v networkctl >/dev/null 2>&1; then
            networkctl reconfigure "$iface" >/dev/null 2>&1 || true
        fi
    fi
    disable_powersave "$iface"
}

bring_down_iface() {
    local iface="$1"
    [ -d "/sys/class/net/$iface" ] || return 0

    if is_nm_active; then
        nmcli device disconnect "$iface" >/dev/null 2>&1 || true
        ip link set dev "$iface" down 2>/dev/null || true
    else
        local wpa_unit="netplan-wpa-${iface}.service"
        if systemctl is-active --quiet "$wpa_unit" 2>/dev/null; then
            systemctl stop "$wpa_unit" >/dev/null 2>&1 || true
        fi
        ip link set dev "$iface" down 2>/dev/null || true
        ip addr flush dev "$iface" 2>/dev/null || true
    fi
}

SECONDS=100
USB_WAS_PRESENT=false
USB_ONLINE_SINCE=0
USB_OFFLINE_SINCE=0
LAST_USB_RECONFIG=0
LAST_INT_RECONFIG=$SECONDS
NETPLAN_GEN_TRIED=false

log "Watchdog started (USB=${USB_IF}, onboard=${INT_IF}, failback=${FAILBACK_SEC}s)."

while true; do
    NOW=$SECONDS

    if iface_exists "$USB_IF"; then
        if [ "$USB_WAS_PRESENT" = false ]; then
            log "External USB Wi-Fi (${USB_IF}) detected."
            USB_WAS_PRESENT=true
            USB_ONLINE_SINCE=0
            USB_OFFLINE_SINCE=$NOW
            LAST_USB_RECONFIG=0
        fi

        # Ensure Netplan generated netplan-wpa-wlusb0.service (handles hotplug before first reboot)
        if ! is_nm_active && [ "$NETPLAN_GEN_TRIED" = false ]; then
            if ! systemctl cat "netplan-wpa-${USB_IF}.service" >/dev/null 2>&1; then
                log "Generating Netplan units for ${USB_IF}..."
                netplan generate >/dev/null 2>&1 || true
                systemctl daemon-reload >/dev/null 2>&1 || true
                networkctl reload >/dev/null 2>&1 || true
            fi
            NETPLAN_GEN_TRIED=true
        fi

        if ! iface_is_online "$USB_IF"; then
            if (( NOW - LAST_USB_RECONFIG >= RECONFIG_RETRY_SEC )); then
                bring_up_iface "$USB_IF" true
                LAST_USB_RECONFIG=$NOW
            else
                bring_up_iface "$USB_IF" false
            fi
        else
            disable_powersave "$USB_IF"
        fi

        if iface_is_online "$USB_IF"; then
            USB_OFFLINE_SINCE=0
            if (( USB_ONLINE_SINCE == 0 )); then
                USB_ONLINE_SINCE=$NOW
            fi

            if (( NOW - USB_ONLINE_SINCE >= STABLE_HANDOVER_SEC )); then
                if iface_exists "$INT_IF" && iface_is_up_or_wpa_active "$INT_IF"; then
                    USB_IP=$(iface_ipv4 "$USB_IF")
                    log "USB Wi-Fi (${USB_IF}) stably online (${USB_IP}) — disabling onboard ${INT_IF}."
                    bring_down_iface "$INT_IF"
                fi
            fi
        else
            USB_ONLINE_SINCE=0
            if (( USB_OFFLINE_SINCE == 0 )); then
                USB_OFFLINE_SINCE=$NOW
            fi

            if iface_exists "$INT_IF"; then
                if ! iface_is_up_or_wpa_active "$INT_IF"; then
                    if (( NOW - USB_OFFLINE_SINCE >= FAILBACK_SEC )); then
                        log "USB Wi-Fi (${USB_IF}) offline for $((NOW - USB_OFFLINE_SINCE))s — failing back to onboard ${INT_IF}."
                        bring_up_iface "$INT_IF" true
                        LAST_INT_RECONFIG=$NOW
                    fi
                else
                    if ! iface_is_online "$INT_IF" && (( NOW - LAST_INT_RECONFIG >= RECONFIG_RETRY_SEC )); then
                        bring_up_iface "$INT_IF" true
                        LAST_INT_RECONFIG=$NOW
                    else
                        disable_powersave "$INT_IF"
                    fi
                fi
            fi
        fi
    else
        if [ "$USB_WAS_PRESENT" = true ]; then
            log "External USB Wi-Fi (${USB_IF}) removed — switching immediately to onboard ${INT_IF}."
            USB_WAS_PRESENT=false
            USB_ONLINE_SINCE=0
            USB_OFFLINE_SINCE=0
            LAST_USB_RECONFIG=0
            LAST_INT_RECONFIG=0
            if ! is_nm_active; then
                systemctl stop "netplan-wpa-${USB_IF}.service" >/dev/null 2>&1 || true
            fi
        fi

        if iface_exists "$INT_IF"; then
            if ! iface_is_up_or_wpa_active "$INT_IF"; then
                log "Restoring onboard Wi-Fi (${INT_IF})..."
                bring_up_iface "$INT_IF" true
                LAST_INT_RECONFIG=$NOW
            elif ! iface_is_online "$INT_IF" && (( NOW - LAST_INT_RECONFIG >= RECONFIG_RETRY_SEC )); then
                bring_up_iface "$INT_IF" true
                LAST_INT_RECONFIG=$NOW
            else
                disable_powersave "$INT_IF"
            fi
        fi
    fi

    sleep "$POLL_SEC" &
    SLEEP_PID=$!
    wait "$SLEEP_PID" 2>/dev/null || true
    SLEEP_PID=""
done
EOF
chmod 755 /usr/local/sbin/wifi-failover-watchdog.sh

cat <<'EOF' > /etc/systemd/system/wifi-failover.service
[Unit]
Description=ConeRobot Automatic USB Wi-Fi Failover & Power-Save Watchdog
After=systemd-networkd.service NetworkManager.service network.target

[Service]
Type=simple
ExecStart=/usr/local/sbin/wifi-failover-watchdog.sh
Restart=always
RestartSec=3
KillMode=process

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable wifi-failover.service >/dev/null 2>&1

# ------------------------------------------------------------------------------
# Step 5/6: Generate & atomically validate Netplan YAML (preserving saved SSIDs)
# ------------------------------------------------------------------------------
echo "⚙️ [5/6] Updating Netplan configuration (${NETPLAN_FILE})..."

RENDERER="networkd"
if systemctl is-active --quiet NetworkManager 2>/dev/null && ! systemctl is-active --quiet systemd-networkd 2>/dev/null; then
    RENDERER="NetworkManager"
fi

TMP_NETPLAN="$(mktemp /tmp/netplan-50-cloud-init.XXXXXX.yaml)"
trap 'rm -f "$TMP_NETPLAN"' EXIT

NETPLAN_FILE="$NETPLAN_FILE" \
TMP_NETPLAN="$TMP_NETPLAN" \
REQ_SSID="$REQ_SSID" \
REQ_PASS="$REQ_PASS" \
REPLACE_ALL="$REPLACE_ALL" \
RENDERER="$RENDERER" \
python3 - <<'PYEOF'
import copy
import os
import sys
import yaml

netplan_file = os.environ["NETPLAN_FILE"]
tmp_file = os.environ["TMP_NETPLAN"]
req_ssid = os.environ["REQ_SSID"]
req_pass = os.environ["REQ_PASS"]
replace_all = os.environ["REPLACE_ALL"].lower() == "true"
renderer = os.environ["RENDERER"]

aps = {}
if os.path.isfile(netplan_file) and not replace_all:
    try:
        with open(netplan_file, "r", encoding="utf-8") as f:
            existing = yaml.safe_load(f) or {}
        wifis = (existing.get("network") or {}).get("wifis") or {}
        # Collect saved access-points from any existing Wi-Fi interface (wlusb0, wlan1, wlan0, etc.)
        for iface_cfg in wifis.values():
            if not isinstance(iface_cfg, dict):
                continue
            iface_aps = iface_cfg.get("access-points") or {}
            if not isinstance(iface_aps, dict):
                continue
            for ssid, ap_cfg in iface_aps.items():
                if not ssid:
                    continue
                ssid_str = str(ssid)
                if isinstance(ap_cfg, dict) and ap_cfg:
                    aps.setdefault(ssid_str, copy.deepcopy(ap_cfg))
                elif ap_cfg is None:
                    aps.setdefault(ssid_str, {})
    except Exception as e:
        print(f"   ⚠️ Could not parse existing {netplan_file} ({e}); building clean config.", file=sys.stderr)

if req_ssid:
    entry = aps.get(req_ssid, {})
    if not isinstance(entry, dict):
        entry = {}
    entry["password"] = req_pass
    aps[req_ssid] = entry
elif not aps:
    aps["Bandi"] = {"password": "1234445678"}

config = {
    "network": {
        "version": 2,
        "renderer": renderer,
        "ethernets": {
            "eth0": {
                "optional": True,
                "dhcp4": True,
                "dhcp4-overrides": {
                    "route-metric": 100,
                    "use-hostname": False,
                },
            }
        },
        "wifis": {
            "wlusb0": {
                "optional": True,
                "dhcp4": True,
                "dhcp4-overrides": {
                    "route-metric": 200,
                    "use-hostname": False,
                },
                "access-points": copy.deepcopy(aps),
            },
            "wlan0": {
                "optional": True,
                "dhcp4": True,
                "dhcp4-overrides": {
                    "route-metric": 600,
                    "use-hostname": False,
                },
                "access-points": copy.deepcopy(aps),
            },
        },
    }
}

with open(tmp_file, "w", encoding="utf-8") as f:
    yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False, allow_unicode=True)

print("   ✓ Configured SSIDs (" + renderer + "): " + ", ".join(aps.keys()))
PYEOF

chmod 600 "$TMP_NETPLAN"

# Warn if any other .yaml file in /etc/netplan/ also configures Wi-Fi interfaces
for other_yaml in /etc/netplan/*.yaml; do
    [ -e "$other_yaml" ] || continue
    [ "$other_yaml" = "$NETPLAN_FILE" ] && continue
    if grep -qE '^\s*(wlan0|wlan1|wlusb0):' "$other_yaml" 2>/dev/null; then
        echo "   ⚠️ Warning: ${other_yaml} also references wlan0/wlan1/wlusb0 and may conflict with ${NETPLAN_FILE}."
    fi
done

NETPLAN_CHANGED=true
if [ -f "$NETPLAN_FILE" ] && cmp -s "$TMP_NETPLAN" "$NETPLAN_FILE"; then
    NETPLAN_CHANGED=false
    echo "   ✓ Netplan config already up to date (no changes needed)."
else
    mkdir -p /etc/netplan
    # Keep a one-time backup of the very first pre-script Netplan file
    if [ -f "$NETPLAN_FILE" ] && [ ! -f "${NETPLAN_FILE}.orig.bak" ]; then
        cp -p "$NETPLAN_FILE" "${NETPLAN_FILE}.orig.bak"
        chmod 600 "${NETPLAN_FILE}.orig.bak"
    fi
    ROLLBACK_BAK=""
    if [ -f "$NETPLAN_FILE" ]; then
        ROLLBACK_BAK="${NETPLAN_FILE}.prev.bak"
        cp -p "$NETPLAN_FILE" "$ROLLBACK_BAK"
    fi

    install -m 600 "$TMP_NETPLAN" "$NETPLAN_FILE"

    if ! netplan generate; then
        echo "❌ netplan generate failed! Rolling back ${NETPLAN_FILE}..."
        if [ -n "$ROLLBACK_BAK" ] && [ -f "$ROLLBACK_BAK" ]; then
            mv -f "$ROLLBACK_BAK" "$NETPLAN_FILE"
        fi
        exit 1
    fi
    [ -n "$ROLLBACK_BAK" ] && rm -f "$ROLLBACK_BAK"
    systemctl daemon-reload
    # Reload networkd configs in memory without dropping active SSH sessions
    if [ "$RENDERER" = "networkd" ] && command -v networkctl >/dev/null 2>&1; then
        networkctl reload >/dev/null 2>&1 || true
    fi
fi

# Prevent cloud-init from ever overwriting /etc/netplan/50-cloud-init.yaml on a fresh Pi
if [ -d /etc/cloud/cloud.cfg.d ]; then
    echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
fi

# ------------------------------------------------------------------------------
# Step 6/6: Apply runtime power-save & activate safely (SSH-Safe)
# ------------------------------------------------------------------------------
echo "⚙️ [6/6] Applying runtime settings (SSH-Safe)..."

# Disable power save immediately on all currently present wireless interfaces
for net_path in /sys/class/net/*; do
    [ -d "$net_path/wireless" ] || continue
    iface=$(basename "$net_path")
    iw dev "$iface" set power_save off 2>/dev/null || true
    if readlink -f "$net_path/device" 2>/dev/null | grep -q "/usb"; then
        usb_pwr="$net_path/device/../power/control"
        if [ -w "$usb_pwr" ]; then
            echo "on" > "$usb_pwr" 2>/dev/null || true
        fi
    fi
done

CURRENT_USB_IF="$(detect_usb_wifi_iface || true)"
NEEDS_REPLUG=false
if [ -n "$CURRENT_USB_IF" ] && [ "$CURRENT_USB_IF" != "wlusb0" ]; then
    NEEDS_REPLUG=true
fi

IS_SSH=false
if [ -n "${SSH_CONNECTION:-}" ] || [ -n "${SSH_CLIENT:-}" ] || [ -n "${SSH_TTY:-}" ]; then
    IS_SSH=true
elif command -v pstree >/dev/null 2>&1 && pstree -s $$ 2>/dev/null | grep -q "sshd"; then
    IS_SSH=true
fi

if [ "$IS_SSH" = true ]; then
    echo "   -> Active SSH session detected: preserving current link so SSH does not disconnect."
    if [ "$NEEDS_REPLUG" = false ] && { [ "$NETPLAN_CHANGED" = false ] || [ -z "$CURRENT_USB_IF" ]; }; then
        systemctl restart wifi-failover.service 2>/dev/null || true
        echo "   -> wifi-failover.service is running live."
    else
        echo "   -> wifi-failover.service enabled for next boot / dongle replug."
    fi
    if [ "$NEEDS_REPLUG" = true ]; then
        echo "   👉 USB Wi-Fi is currently named '${CURRENT_USB_IF}'. Unplug & replug it (or run 'sudo reboot') to rename it to 'wlusb0'."
    elif [ "$NETPLAN_CHANGED" = true ]; then
        echo "   👉 Run 'sudo reboot' to apply the updated Netplan Wi-Fi credentials cleanly."
    fi
else
    if [ "$NETPLAN_CHANGED" = true ]; then
        netplan apply 2>/dev/null || true
    fi
    systemctl restart wifi-failover.service 2>/dev/null || true
    if [ "$NEEDS_REPLUG" = true ]; then
        echo "   👉 USB Wi-Fi is currently named '${CURRENT_USB_IF}'. Unplug & replug it (or reboot) to rename it to 'wlusb0'."
    fi
fi

echo ""
echo "=================================================================="
echo " ✅ Wi-Fi Failover & Stability Setup Complete!"
echo "=================================================================="
echo "Current Wireless Interfaces:"
for net_path in /sys/class/net/*; do
    [ -d "$net_path/wireless" ] || continue
    iface=$(basename "$net_path")
    ip4=$(ip -4 -o addr show dev "$iface" scope global 2>/dev/null | awk '{print $4}' | head -n 1 || true)
    ps=$(iw dev "$iface" get power_save 2>/dev/null | awk '{print $NF}' || true)
    ps="${ps:-n/a}"
    echo "  - ${iface}: IP=${ip4:-none} | power_save=${ps}"
done
echo ""
echo "Useful commands:"
echo "  journalctl -u wifi-failover.service -f   # Live failover logs"
echo "  iw dev wlusb0 link                       # Check TP-Link signal"
echo "  iw dev wlan0 link                        # Check onboard Wi-Fi signal"
echo "=================================================================="
