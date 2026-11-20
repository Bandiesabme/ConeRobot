#!/usr/bin/env bash
# ==============================================================================
# ConeRobot Raspberry Pi 5 — Comprehensive Read-Only Setup Health Checker
# ==============================================================================
# Audits every component configured by setup_robot_rpi5.sh, setup_wifi.sh, and
# install_ydlidar.sh without modifying any system files or disrupting running
# ROS 2 nodes.
#
# Usage:
#   bash scripts/check_rpi5_setup.sh          # Standard check (as normal user)
#   sudo bash scripts/check_rpi5_setup.sh     # Full check (also inspects Netplan YAML)
#   bash scripts/check_rpi5_setup.sh --quick  # Skip live I2C/UART hardware probes
# ==============================================================================

set -u

QUICK_MODE=false
for arg in "$@"; do
    case "$arg" in
        --quick|-q) QUICK_MODE=true ;;
        --help|-h)
            echo "Usage: [sudo] bash scripts/check_rpi5_setup.sh [--quick]"
            exit 0
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET_USER="${SUDO_USER:-${USER:-$(id -un)}}"
TARGET_HOME="$(getent passwd "$TARGET_USER" 2>/dev/null | cut -d: -f6 || true)"
TARGET_HOME="${TARGET_HOME:-$HOME}"

# Colors (disabled cleanly if not a TTY)
if [ -t 1 ]; then
    BOLD="\033[1m"
    GREEN="\033[0;32m"
    YELLOW="\033[0;33m"
    RED="\033[0;31m"
    CYAN="\033[0;36m"
    DIM="\033[2m"
    RESET="\033[0m"
else
    BOLD="" GREEN="" YELLOW="" RED="" CYAN="" DIM="" RESET=""
fi

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
FAIL_ITEMS=()
WARN_ITEMS=()

section() {
    echo ""
    echo -e "${CYAN}${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${CYAN}${BOLD} $1${RESET}"
    echo -e "${CYAN}${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

pass() {
    PASS_COUNT=$((PASS_COUNT + 1))
    echo -e "  ${GREEN}[PASS]${RESET} $1"
}

warn() {
    local msg="$1"
    local fix="${2:-}"
    WARN_COUNT=$((WARN_COUNT + 1))
    echo -e "  ${YELLOW}[WARN]${RESET} $msg"
    if [ -n "$fix" ]; then
        echo -e "         ${DIM}↳ Fix: ${fix}${RESET}"
        WARN_ITEMS+=("${msg}  ->  ${fix}")
    else
        WARN_ITEMS+=("${msg}")
    fi
}

fail() {
    local msg="$1"
    local fix="${2:-}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    echo -e "  ${RED}[FAIL]${RESET} $msg"
    if [ -n "$fix" ]; then
        echo -e "         ${DIM}↳ Fix: ${fix}${RESET}"
        FAIL_ITEMS+=("${msg}  ->  ${fix}")
    else
        FAIL_ITEMS+=("${msg}")
    fi
}

info() {
    echo -e "  ${DIM}[INFO]${RESET} $1"
}

can_target_rw() {
    local p="$1"
    if [ "$EUID" -eq 0 ] && [ "$TARGET_USER" != "root" ] && command -v runuser >/dev/null 2>&1; then
        runuser -u "$TARGET_USER" -- test -r "$p" -a -w "$p" 2>/dev/null
    else
        [ -r "$p" ] && [ -w "$p" ]
    fi
}

echo -e "${BOLD}====================================================================${RESET}"
echo -e "${BOLD} 🤖 ConeRobot Raspberry Pi 5 Setup Verification${RESET}"
echo -e "${BOLD}====================================================================${RESET}"
info "Workspace   : ${WORKSPACE_DIR}"
info "Target User : ${TARGET_USER} (home: ${TARGET_HOME})"
info "Run Context : $( [ "$EUID" -eq 0 ] && echo "root/sudo (full inspection)" || echo "non-root (run with sudo to also inspect /etc/netplan YAML)" )"
info "Timestamp   : $(date '+%Y-%m-%d %H:%M:%S')"

# ==============================================================================
# 1. SYSTEM, OS, SWAP & POWER HEALTH
# ==============================================================================
section "1. System, OS, Swap Memory & Power Health"

if [ -r /proc/device-tree/model ]; then
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
    if echo "$MODEL" | grep -qi "Raspberry Pi 5"; then
        pass "Hardware: ${MODEL}"
    else
        warn "Hardware is not Raspberry Pi 5: ${MODEL}"
    fi
else
    warn "Cannot read /proc/device-tree/model (not running on a Raspberry Pi?)"
fi

if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    OS_NAME=$(. /etc/os-release && echo "${PRETTY_NAME:-unknown}")
    OS_CODENAME=$(. /etc/os-release && echo "${VERSION_CODENAME:-unknown}")
    if [ "$OS_CODENAME" = "noble" ]; then
        pass "OS: ${OS_NAME}"
    else
        warn "OS is '${OS_NAME}' (expected Ubuntu 24.04 noble for ROS 2 Jazzy)"
    fi
fi

# RAM & Swap (critical on 1 GB Pi 5)
TOTAL_RAM_MB=$(awk '/MemTotal:/ {printf "%d", $2/1024}' /proc/meminfo 2>/dev/null || echo 0)
info "Total RAM: ${TOTAL_RAM_MB} MB"

SWAP_LINE=$(swapon --show --bytes --noheadings 2>/dev/null | grep "/swapfile" || true)
if [ -n "$SWAP_LINE" ]; then
    SWAP_BYTES=$(echo "$SWAP_LINE" | awk '{print $3}')
    SWAP_MB=$((SWAP_BYTES / 1024 / 1024))
    if [ "$SWAP_MB" -ge 1800 ]; then
        pass "Swap active: /swapfile (${SWAP_MB} MB)"
    else
        warn "Swap /swapfile is small (${SWAP_MB} MB; recommended >= 2048 MB for 1 GB Pi 5)" \
             "bash scripts/setup_robot_rpi5.sh (Step 1)"
    fi
else
    fail "Swapfile (/swapfile) is NOT active — colcon builds will OOM on 1 GB Pi 5" \
         "sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile"
fi

if grep -q "/swapfile" /etc/fstab 2>/dev/null; then
    pass "Swap persistence: /swapfile entry present in /etc/fstab"
else
    fail "Swapfile is missing from /etc/fstab (will not survive reboot)" \
         "echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
fi

# Disk space
DISK_AVAIL_MB=$(df -Pm / 2>/dev/null | awk 'NR==2 {print $4}' || echo 0)
if [ "$DISK_AVAIL_MB" -ge 2048 ]; then
    pass "Root disk free space: ${DISK_AVAIL_MB} MB"
else
    warn "Low disk space on /: ${DISK_AVAIL_MB} MB free (< 2 GB)"
fi

# Thermal & Power (Pi 5 vcgencmd)
if command -v vcgencmd >/dev/null 2>&1; then
    THROTTLED=$(vcgencmd get_throttled 2>/dev/null | awk -F= '{print $2}' || true)
    if [ "$THROTTLED" = "0x0" ]; then
        pass "Power / Throttling: ${THROTTLED} (no undervoltage or throttling)"
    elif [ -n "$THROTTLED" ]; then
        warn "Power / Throttling flags detected: ${THROTTLED} (check 5V PSU / buck converter!)"
    fi

    TEMP_STR=$(vcgencmd measure_temp 2>/dev/null | awk -F= '{print $2}' || true)
    TEMP_INT=$(echo "$TEMP_STR" | sed 's/[^0-9.].*//' | awk '{printf "%d", $1}' 2>/dev/null || echo 0)
    if [ -n "$TEMP_STR" ]; then
        if [ "$TEMP_INT" -lt 75 ]; then
            pass "SoC Temperature: ${TEMP_STR}"
        else
            warn "High SoC Temperature: ${TEMP_STR}"
        fi
    fi

    EXT5V=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | grep -oE '[0-9]+\.[0-9]+V' | head -n 1 || true)
    if [ -n "$EXT5V" ]; then
        EXT5V_NUM=$(echo "$EXT5V" | tr -d 'V')
        if awk -v v="$EXT5V_NUM" 'BEGIN { exit !(v >= 4.90) }'; then
            pass "Pi 5 EXT5V rail: ${EXT5V}"
        else
            warn "Pi 5 EXT5V rail is low: ${EXT5V} (want >= 4.90V)"
        fi
    fi
else
    info "vcgencmd not available (skipping PMIC/throttling check)"
fi

# ==============================================================================
# 2. BOOT FIRMWARE CONFIGURATION
# ==============================================================================
section "2. Boot Firmware Configuration (/boot/firmware/config.txt & cmdline.txt)"

BOOT_CONFIG="/boot/firmware/config.txt"
if [ -r "$BOOT_CONFIG" ]; then
    if grep -qE '^\s*usb_max_current_enable=1\b' "$BOOT_CONFIG"; then
        pass "usb_max_current_enable=1 present (1.6A USB power for LiDAR & Wi-Fi dongle)"
    else
        fail "usb_max_current_enable=1 missing in ${BOOT_CONFIG}" \
             "echo 'usb_max_current_enable=1' | sudo tee -a ${BOOT_CONFIG} && sudo reboot"
    fi

    if grep -qE '^\s*dtparam=i2c_arm=on' "$BOOT_CONFIG"; then
        if grep -qE '^\s*dtparam=i2c_arm=on.*i2c_arm_baudrate=400000' "$BOOT_CONFIG"; then
            pass "400 kHz fast I2C configured (dtparam=i2c_arm=on,i2c_arm_baudrate=400000)"
        else
            warn "I2C is enabled, but 400 kHz baudrate not explicitly set in ${BOOT_CONFIG}" \
                 "Set dtparam=i2c_arm=on,i2c_arm_baudrate=400000 in ${BOOT_CONFIG}"
        fi
    else
        fail "I2C bus not enabled (dtparam=i2c_arm=on missing in ${BOOT_CONFIG})" \
             "echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' | sudo tee -a ${BOOT_CONFIG} && sudo reboot"
    fi

    if grep -qE '^\s*dtparam=uart0=on\b' "$BOOT_CONFIG" && grep -qE '^\s*enable_uart=1\b' "$BOOT_CONFIG"; then
        pass "Hardware UART0 enabled (enable_uart=1 & dtparam=uart0=on)"
    else
        fail "Hardware UART0 not fully enabled in ${BOOT_CONFIG} (need enable_uart=1 and dtparam=uart0=on)" \
             "Add enable_uart=1 and dtparam=uart0=on to ${BOOT_CONFIG} && sudo reboot"
    fi
else
    fail "${BOOT_CONFIG} not found or not readable"
fi

BOOT_CMDLINE="/boot/firmware/cmdline.txt"
if [ -r "$BOOT_CMDLINE" ]; then
    if grep -qE 'console=(serial0|ttyAMA0)' "$BOOT_CMDLINE"; then
        fail "Kernel serial console active on ttyAMA0/serial0 in ${BOOT_CMDLINE} (conflicts with LC29H GPS!)" \
             "Remove console=serial0,... / console=ttyAMA0,... from ${BOOT_CMDLINE} and reboot"
    else
        pass "No serial console grabbing ttyAMA0/serial0 in ${BOOT_CMDLINE}"
    fi
fi

# ==============================================================================
# 3. USER GROUPS, PERMISSIONS & SERIAL ISOLATION
# ==============================================================================
section "3. User Groups, Hardware Permissions & Serial Isolation"

USER_GROUPS=" $(id -nG "$TARGET_USER" 2>/dev/null || true) "
for grp in i2c dialout; do
    if echo "$USER_GROUPS" | grep -q " ${grp} "; then
        pass "User '${TARGET_USER}' is in group '${grp}'"
    else
        fail "User '${TARGET_USER}' is NOT in group '${grp}'" \
             "sudo usermod -aG ${grp} ${TARGET_USER} (then log out & back in)"
    fi
done

for rule in /etc/udev/rules.d/99-gpio.rules /etc/udev/rules.d/99-i2c.rules /etc/udev/rules.d/ydlidar.rules; do
    if [ -f "$rule" ]; then
        pass "Udev rule present: ${rule}"
    else
        if [ "$rule" = "/etc/udev/rules.d/ydlidar.rules" ]; then
            warn "Udev rule missing: ${rule} (required for LiDAR robots)" \
                 "bash scripts/init_ydlidar_udev.sh"
        else
            fail "Udev rule missing: ${rule}" \
                 "bash scripts/setup_robot_rpi5.sh (Step 3)"
        fi
    fi
done

# Check device node existence and access (tested as TARGET_USER even under sudo)
if [ -e /dev/i2c-1 ]; then
    if can_target_rw /dev/i2c-1; then
        pass "/dev/i2c-1 exists and is readable/writable by '${TARGET_USER}'"
    else
        fail "/dev/i2c-1 exists but '${TARGET_USER}' lacks R/W permissions ($(ls -l /dev/i2c-1 | awk '{print $1,$3,$4}'))" \
             "sudo usermod -aG i2c ${TARGET_USER} && sudo udevadm trigger"
    fi
else
    fail "/dev/i2c-1 does not exist (reboot after enabling dtparam=i2c_arm=on)"
fi

if [ -e /dev/ttyAMA0 ]; then
    if can_target_rw /dev/ttyAMA0; then
        pass "/dev/ttyAMA0 exists and is readable/writable by '${TARGET_USER}'"
    else
        fail "/dev/ttyAMA0 exists but '${TARGET_USER}' lacks R/W permissions ($(ls -l /dev/ttyAMA0 | awk '{print $1,$3,$4}'))" \
             "sudo usermod -aG dialout ${TARGET_USER} (then re-login)"
    fi
else
    fail "/dev/ttyAMA0 does not exist (reboot after enabling dtparam=uart0=on)"
fi

GPIO_CHIPS=(/dev/gpiochip*)
if [ -e "${GPIO_CHIPS[0]}" ]; then
    GPIO_BAD=""
    for chip in "${GPIO_CHIPS[@]}"; do
        if ! can_target_rw "$chip"; then
            GPIO_BAD="${GPIO_BAD} ${chip}"
        fi
    done
    if [ -z "$GPIO_BAD" ]; then
        pass "GPIO chips accessible by '${TARGET_USER}' (${#GPIO_CHIPS[@]} chip(s))"
    else
        fail "GPIO chip(s) lack R/W permissions for '${TARGET_USER}':${GPIO_BAD}" \
             "sudo chmod 666 /dev/gpiochip* && sudo udevadm trigger"
    fi
else
    fail "No /dev/gpiochip* device found"
fi

# Serial-getty & gpsd isolation
for svc in serial-getty@ttyAMA0.service gpsd.service gpsd.socket; do
    if systemctl is-active --quiet "$svc" 2>/dev/null; then
        fail "${svc} is ACTIVE (will corrupt /dev/ttyAMA0 GPS stream!)" \
             "sudo systemctl stop ${svc} && sudo systemctl mask ${svc}"
    else
        STATE=$(systemctl is-enabled "$svc" 2>/dev/null || true)
        STATE="${STATE:-not-found}"
        pass "${svc} isolated (active=no, state=${STATE})"
    fi
done

# ==============================================================================
# 4. WI-FI FAILOVER, TP-LINK TL-WN722N v2 & NETWORKING
# ==============================================================================
section "4. Wi-Fi Failover, TP-Link USB Dongle & Networking"

# Check no leftover legacy switcher artifacts
LEGACY_FOUND=()
for f in /etc/systemd/system/wifi-dongle-switch@.service \
         /usr/local/bin/wifi-dongle-switcher.sh \
         /etc/udev/rules.d/70-wifi-naming.rules \
         /etc/udev/rules.d/99-wifi-dongle.rules; do
    [ -e "$f" ] && LEGACY_FOUND+=("$f")
done
if [ "${#LEGACY_FOUND[@]}" -eq 0 ]; then
    pass "No legacy Wi-Fi switcher artifacts left behind"
else
    fail "Old Wi-Fi switcher files still exist: ${LEGACY_FOUND[*]}" \
         "sudo bash scripts/setup_wifi.sh && sudo reboot"
fi

for f in /etc/systemd/network/10-wifi-usb-dongle.link \
         /etc/systemd/network/11-wifi-onboard.link \
         /etc/udev/rules.d/70-wifi-usb-dongle.rules \
         /etc/udev/rules.d/99-wifi-failover.rules \
         /etc/sysctl.d/99-wifi-failover.conf \
         /usr/local/sbin/wifi-failover-watchdog.sh \
         /etc/systemd/system/wifi-failover.service; do
    if [ -f "$f" ]; then
        pass "Wi-Fi failover file installed: ${f}"
    else
        fail "Wi-Fi failover file missing: ${f}" \
             "sudo bash scripts/setup_wifi.sh"
    fi
done

# Cloud-init network overwrite guard
if [ -d /etc/cloud/cloud.cfg.d ]; then
    if [ -f /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg ]; then
        pass "cloud-init network overwrite disabled (/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg)"
    else
        warn "cloud-init network guard file missing (re-run setup_wifi.sh to add)" \
             "sudo bash scripts/setup_wifi.sh"
    fi
fi

# Anti-ARP-flux live kernel sysctl
ARP_IGN=$(cat /proc/sys/net/ipv4/conf/all/arp_ignore 2>/dev/null || echo "?")
ARP_ANN=$(cat /proc/sys/net/ipv4/conf/all/arp_announce 2>/dev/null || echo "?")
if [ "$ARP_IGN" = "1" ] && [ "$ARP_ANN" = "2" ]; then
    pass "Anti-ARP-flux sysctl active (arp_ignore=${ARP_IGN}, arp_announce=${ARP_ANN})"
else
    warn "Anti-ARP-flux sysctl not yet active (arp_ignore=${ARP_IGN}, arp_announce=${ARP_ANN})" \
         "sudo sysctl -p /etc/sysctl.d/99-wifi-failover.conf"
fi

# Watchdog service state
WF_ENABLED=$(systemctl is-enabled wifi-failover.service 2>/dev/null || true)
WF_ENABLED="${WF_ENABLED:-missing}"
WF_ACTIVE=$(systemctl is-active wifi-failover.service 2>/dev/null || true)
WF_ACTIVE="${WF_ACTIVE:-inactive}"
if [ "$WF_ENABLED" = "enabled" ] && [ "$WF_ACTIVE" = "active" ]; then
    pass "wifi-failover.service is enabled and running"
elif [ "$WF_ENABLED" = "enabled" ]; then
    warn "wifi-failover.service is enabled but not currently active (state=${WF_ACTIVE})" \
         "sudo systemctl start wifi-failover.service"
else
    fail "wifi-failover.service is not enabled/running (enabled=${WF_ENABLED}, active=${WF_ACTIVE})" \
         "sudo bash scripts/setup_wifi.sh"
fi

# Netplan check
NETPLAN_FILE="/etc/netplan/50-cloud-init.yaml"
if [ -e "$NETPLAN_FILE" ]; then
    NP_PERMS=$(stat -c '%a' "$NETPLAN_FILE" 2>/dev/null || echo "?")
    if [ "$NP_PERMS" = "600" ]; then
        pass "Netplan file exists with secure permissions (${NETPLAN_FILE}, mode ${NP_PERMS})"
    else
        warn "Netplan permissions are ${NP_PERMS} (recommended 600)" \
             "sudo chmod 600 ${NETPLAN_FILE}"
    fi

    if [ -r "$NETPLAN_FILE" ] && python3 -c "import yaml" >/dev/null 2>&1; then
        NP_SUMMARY=$(python3 - "$NETPLAN_FILE" <<'PYEOF' 2>&1
import sys, yaml
with open(sys.argv[1], "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
net = cfg.get("network") or {}
wifis = net.get("wifis") or {}
has_usb = "wlusb0" in wifis
has_int = "wlan0" in wifis
ssids = sorted({str(k) for v in wifis.values() if isinstance(v, dict) for k in (v.get("access-points") or {})})
renderer = net.get("renderer", "networkd(default)")
print(f"OK|{renderer}|{has_usb}|{has_int}|{','.join(ssids)}")
PYEOF
        )
        if echo "$NP_SUMMARY" | grep -q '^OK|'; then
            IFS='|' read -r _ NP_REND NP_USB NP_INT NP_SSIDS <<< "$NP_SUMMARY"
            if [ "$NP_USB" = "True" ] && [ "$NP_INT" = "True" ]; then
                pass "Netplan YAML valid (renderer=${NP_REND}, wlusb0+wlan0 configured, SSIDs: ${NP_SSIDS:-none})"
            else
                fail "Netplan YAML missing wlusb0 or wlan0 (wlusb0=${NP_USB}, wlan0=${NP_INT})" \
                     "sudo bash scripts/setup_wifi.sh"
            fi
        else
            fail "Netplan YAML parse error: ${NP_SUMMARY}" \
                 "sudo bash scripts/setup_wifi.sh"
        fi
    else
        info "Run with sudo to inspect ${NETPLAN_FILE} contents (mode 600 root-only)."
    fi
else
    fail "${NETPLAN_FILE} missing" "sudo bash scripts/setup_wifi.sh"
fi

# Check for conflicting extra netplan files
for other_yaml in /etc/netplan/*.yaml; do
    [ -e "$other_yaml" ] || continue
    [ "$other_yaml" = "$NETPLAN_FILE" ] && continue
    if [ -r "$other_yaml" ] && grep -qE '^\s*(wlan0|wlan1|wlusb0):' "$other_yaml" 2>/dev/null; then
        warn "Extra Netplan file ${other_yaml} also configures Wi-Fi and may conflict with ${NETPLAN_FILE}"
    fi
done

# RTL8188EU firmware for TP-Link TL-WN722N v2/v3
if ls /lib/firmware/rtlwifi/rtl8188eufw.bin* /usr/lib/firmware/rtlwifi/rtl8188eufw.bin* >/dev/null 2>&1; then
    pass "RTL8188EU firmware present (TP-Link TL-WN722N v2/v3 supported)"
else
    warn "RTL8188EU firmware not found (required for TP-Link TL-WN722N v2/v3)" \
         "sudo apt-get update && sudo apt-get install -y linux-firmware"
fi

# Live USB & wireless interface check
if command -v lsusb >/dev/null 2>&1 && lsusb 2>/dev/null | grep -qi "2357:010c"; then
    info "USB device detected: TP-Link TL-WN722N v2 (2357:010c)"
fi

FOUND_USB_IF=""
ONLINE_WIFI_COUNT=0
for net_path in /sys/class/net/*; do
    [ -d "$net_path/wireless" ] || continue
    iface=$(basename "$net_path")
    drv=$(basename "$(readlink -f "$net_path/device/driver" 2>/dev/null)" 2>/dev/null || echo "unknown")
    is_usb=false
    if readlink -f "$net_path/device" 2>/dev/null | grep -q "/usb"; then
        is_usb=true
        FOUND_USB_IF="$iface"
    fi
    state=$(cat "$net_path/operstate" 2>/dev/null || echo "unknown")
    ip4=$(ip -4 -o addr show dev "$iface" scope global 2>/dev/null | awk '{print $4}' | head -n 1 || true)
    ps=$(iw dev "$iface" get power_save 2>/dev/null | awk '{print $NF}' || true)
    ps="${ps:-n/a}"

    if [ -n "$ip4" ] && [ "$state" = "up" ]; then
        ONLINE_WIFI_COUNT=$((ONLINE_WIFI_COUNT + 1))
    fi

    info "Interface ${iface} (driver=${drv}, usb=${is_usb}): state=${state}, IP=${ip4:-none}, power_save=${ps}"

    if [ "$is_usb" = true ]; then
        if [ "$iface" = "wlusb0" ]; then
            pass "USB Wi-Fi correctly named 'wlusb0' (driver=${drv})"
        else
            fail "USB Wi-Fi is named '${iface}' instead of 'wlusb0'" \
                 "Unplug & replug the TP-Link adapter or run 'sudo reboot'"
        fi
        usb_pwr="$net_path/device/../power/control"
        if [ -r "$usb_pwr" ]; then
            pwr_val=$(cat "$usb_pwr" 2>/dev/null || echo "?")
            if [ "$pwr_val" = "on" ]; then
                pass "USB Wi-Fi autosuspend disabled (power/control=on)"
            else
                warn "USB Wi-Fi autosuspend is '${pwr_val}' (expected 'on')" \
                     "sudo systemctl restart wifi-failover.service"
            fi
        fi
    fi

    if [ "$state" = "up" ]; then
        if [ "$ps" = "off" ] || [ "$ps" = "n/a" ]; then
            pass "${iface} Wi-Fi power_save=${ps}"
        else
            fail "${iface} Wi-Fi power_save is '${ps}' (causes ping spikes & drops)" \
                 "sudo iw dev ${iface} set power_save off && sudo systemctl restart wifi-failover.service"
        fi
    fi
done

if [ -z "$FOUND_USB_IF" ]; then
    info "No external USB Wi-Fi adapter plugged in right now (onboard wlan0 fallback mode)."
fi

if [ "$ONLINE_WIFI_COUNT" -gt 1 ]; then
    warn "Both wlusb0 and wlan0 currently have active IPs (${ONLINE_WIFI_COUNT} Wi-Fi links up — wait 5s or check wifi-failover.service)" \
         "journalctl -u wifi-failover.service -n 20"
fi

DEF_ROUTE=$(ip -4 route show default 2>/dev/null | head -n 1 || true)
if [ -n "$DEF_ROUTE" ]; then
    pass "Default IPv4 route active: ${DEF_ROUTE}"
else
    warn "No default IPv4 route currently active"
fi

# ==============================================================================
# 5. ROS 2 JAZZY & PYTHON HARDWARE LIBRARIES
# ==============================================================================
section "5. ROS 2 Jazzy & Core Packages"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    pass "ROS 2 Jazzy installed (/opt/ros/jazzy/setup.bash)"
else
    fail "ROS 2 Jazzy not found (/opt/ros/jazzy/setup.bash missing)" \
         "bash scripts/setup_robot_rpi5.sh (Step 2)"
fi

REQUIRED_DEBS=(
    ros-jazzy-ros-base
    ros-jazzy-foxglove-bridge
    ros-jazzy-tf2-ros
    ros-jazzy-sensor-msgs
    ros-jazzy-nav-msgs
    python3-colcon-common-extensions
    i2c-tools
    iw
    wpasupplicant
)
MISSING_DEBS=()
for pkg in "${REQUIRED_DEBS[@]}"; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
        MISSING_DEBS+=("$pkg")
    fi
done
if [ "${#MISSING_DEBS[@]}" -eq 0 ]; then
    pass "All ${#REQUIRED_DEBS[@]} core ROS 2 & system APT packages installed"
else
    fail "Missing APT packages: ${MISSING_DEBS[*]}" \
         "sudo apt update && sudo apt install -y ${MISSING_DEBS[*]}"
fi

PY_MODS=(serial lgpio gpiozero smbus yaml board adafruit_bno08x)
MISSING_PY=()
for mod in "${PY_MODS[@]}"; do
    if ! python3 -c "import ${mod}" >/dev/null 2>&1; then
        MISSING_PY+=("$mod")
    fi
done
if [ "${#MISSING_PY[@]}" -eq 0 ]; then
    pass "All Python hardware libraries import cleanly (${PY_MODS[*]})"
else
    fail "Missing/broken Python modules: ${MISSING_PY[*]}" \
         "sudo python3 -m pip install --break-system-packages adafruit-circuitpython-bno08x adafruit-blinka"
fi

# ==============================================================================
# 6. ENVIRONMENT VARIABLES (~/.bashrc)
# ==============================================================================
section "6. Environment Configuration (${TARGET_HOME}/.bashrc)"

TARGET_BASHRC="${TARGET_HOME}/.bashrc"
if [ -r "$TARGET_BASHRC" ]; then
    for pattern_desc in \
        "source /opt/ros/jazzy/setup.bash|ROS 2 Jazzy setup.bash" \
        "ROS_DOMAIN_ID=42|ROS_DOMAIN_ID=42" \
        "GPIOZERO_PIN_FACTORY=lgpio|GPIOZERO_PIN_FACTORY=lgpio" \
        "install/setup.bash|Workspace install/setup.bash"; do
        IFS='|' read -r pat desc <<< "$pattern_desc"
        if grep -q "$pat" "$TARGET_BASHRC"; then
            pass "${desc} configured in ${TARGET_BASHRC}"
        else
            fail "${desc} missing in ${TARGET_BASHRC}" \
                 "bash scripts/setup_robot_rpi5.sh (Step 7)"
        fi
    done
else
    fail "Cannot read ${TARGET_BASHRC}"
fi

# ==============================================================================
# 7. YDLIDAR SDK & ROS 2 WORKSPACE BUILD
# ==============================================================================
section "7. YDLIDAR SDK & ROS 2 Workspace Build"

if ldconfig -p 2>/dev/null | grep -qi "ydlidar" || ls /usr/local/lib/libydlidar_sdk* >/dev/null 2>&1; then
    pass "YDLidar C++ SDK library installed"
else
    warn "YDLidar C++ SDK library not found (required for LiDAR robots)" \
         "bash scripts/install_ydlidar.sh"
fi

for pkg in cone_robot_control cone_robot_interfaces; do
    if [ -d "${WORKSPACE_DIR}/src/${pkg}" ]; then
        pass "Source package present: src/${pkg}"
    else
        fail "Source package missing: src/${pkg}"
    fi
done

for pkg in ydlidar_ros2_driver rf2o_laser_odometry; do
    if [ -d "${WORKSPACE_DIR}/src/${pkg}" ]; then
        pass "Optional LiDAR package present: src/${pkg}"
    else
        warn "Optional LiDAR package missing: src/${pkg} (ok if GPS-only robot)" \
             "bash scripts/install_ydlidar.sh"
    fi
done

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    pass "Workspace built (${WORKSPACE_DIR}/install/setup.bash exists)"

    # Verify ROS 2 package & executable resolution inside a clean subshell (ROS setup uses unset vars)
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        ROS_CHECK=$(
            set +u
            # shellcheck disable=SC1091
            source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
            # shellcheck disable=SC1091
            source "${WORKSPACE_DIR}/install/setup.bash" >/dev/null 2>&1
            missing=""
            for p in cone_robot_control cone_robot_interfaces; do
                ros2 pkg prefix "$p" >/dev/null 2>&1 || missing="${missing} pkg:${p}"
            done
            execs=$(ros2 pkg executables cone_robot_control 2>/dev/null || true)
            for node in mdd10_motor_controller bno08x_node lc29h_gps_node primitive_motion_controller; do
                echo "$execs" | grep -q "$node" || missing="${missing} node:${node}"
            done
            echo "$missing"
        )
        ROS_CHECK=$(echo "$ROS_CHECK" | xargs || true)
        if [ -z "$ROS_CHECK" ]; then
            pass "All cone_robot_control ROS 2 nodes & cone_robot_interfaces resolve cleanly"
        else
            fail "Built workspace missing items: ${ROS_CHECK}" \
                 "cd ${WORKSPACE_DIR} && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --parallel-workers 2"
        fi
    fi
else
    fail "Workspace has not been built yet (${WORKSPACE_DIR}/install/setup.bash missing)" \
         "cd ${WORKSPACE_DIR} && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --parallel-workers 2"
fi

# ==============================================================================
# 8. LIVE HARDWARE PROBES (NON-DESTRUCTIVE)
# ==============================================================================
section "8. Live Hardware Probes (I2C, GPS UART, LiDAR, GPIO)"

if [ "$QUICK_MODE" = true ]; then
    info "Skipped live hardware probes (--quick mode)."
else
    # 8A. I2C bus scan (BNO08x @ 0x4A/0x4B, INA219 @ 0x40)
    if command -v i2cdetect >/dev/null 2>&1 && [ -r /dev/i2c-1 ]; then
        I2C_OUT=$(i2cdetect -y 1 2>/dev/null || true)
        if echo "$I2C_OUT" | grep -qiE '\b(4a|4b)\b'; then
            pass "I2C BNO08x IMU detected on /dev/i2c-1 (addr 0x4A/0x4B)"
        else
            warn "I2C BNO08x IMU NOT detected at 0x4A/0x4B (check 3V3, GND, SDA Pin 3, SCL Pin 5)"
        fi
        if echo "$I2C_OUT" | grep -qiE '\b40\b'; then
            pass "I2C INA219 Battery Monitor detected on /dev/i2c-1 (addr 0x40)"
        else
            warn "I2C INA219 Battery Monitor NOT detected at 0x40 (check wiring if installed)"
        fi
    else
        warn "Skipping I2C bus scan (i2cdetect missing or /dev/i2c-1 inaccessible)"
    fi

    # 8B. GPS UART (/dev/ttyAMA0) — never disturb a running lc29h_gps_node
    if pgrep -f "lc29h_gps_node" >/dev/null 2>&1; then
        pass "lc29h_gps_node is currently running on /dev/ttyAMA0 (skipping raw UART peek)"
    elif [ -r /dev/ttyAMA0 ] && python3 -c "import serial" >/dev/null 2>&1; then
        GPS_PEEK=$(python3 - <<'PYEOF' 2>&1
import serial, time
try:
    s = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.5)
    t0 = time.time()
    nmea = 0
    sample = ""
    while time.time() - t0 < 2.0 and nmea < 5:
        line = s.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('$G') or line.startswith('$P'):
            nmea += 1
            if not sample:
                sample = line[:48]
    s.close()
    if nmea > 0:
        print(f"OK|{nmea}|{sample}")
    else:
        print("NONE")
except Exception as e:
    print(f"ERR|{e}")
PYEOF
        )
        if echo "$GPS_PEEK" | grep -q '^OK|'; then
            IFS='|' read -r _ NMEA_CNT NMEA_SAMPLE <<< "$GPS_PEEK"
            pass "LC29H GPS UART active on /dev/ttyAMA0 @ 115200 (${NMEA_CNT} NMEA lines, e.g. ${NMEA_SAMPLE}...)"
        elif [ "$GPS_PEEK" = "NONE" ]; then
            warn "No NMEA data received on /dev/ttyAMA0 @ 115200 (check GPS HAT seating / switch / power)"
        else
            warn "Could not probe /dev/ttyAMA0 (${GPS_PEEK#ERR|})"
        fi
    else
        warn "Skipping /dev/ttyAMA0 probe (device unreadable or pyserial missing)"
    fi

    # 8C. YDLIDAR USB symlink
    if [ -L /dev/ydlidar ] && [ -e /dev/ydlidar ]; then
        LIDAR_TARGET=$(readlink -f /dev/ydlidar)
        pass "YDLIDAR detected: /dev/ydlidar -> ${LIDAR_TARGET}"
    else
        warn "YDLIDAR not detected at /dev/ydlidar (ok if unplugged or GPS-only robot)"
    fi

    # 8D. lgpio chip open test
    GPIO_TEST=$(python3 - <<'PYEOF' 2>&1
import lgpio
for chip in (4, 0):
    try:
        h = lgpio.gpiochip_open(chip)
        lgpio.gpiochip_close(h)
        print(f"OK|gpiochip{chip}")
        raise SystemExit(0)
    except Exception:
        pass
print("FAIL")
PYEOF
    )
    if echo "$GPIO_TEST" | grep -q '^OK|'; then
        pass "lgpio hardware GPIO access verified (${GPIO_TEST#OK|})"
    else
        fail "lgpio could not open gpiochip4 or gpiochip0 (check /dev/gpiochip* permissions)" \
             "sudo chmod 666 /dev/gpiochip* && sudo udevadm trigger"
    fi
fi

# ==============================================================================
# 9. FINAL SUMMARY
# ==============================================================================
echo ""
echo -e "${BOLD}====================================================================${RESET}"
echo -e "${BOLD} SUMMARY: ${GREEN}${PASS_COUNT} PASS${RESET} | ${YELLOW}${WARN_COUNT} WARN${RESET} | ${RED}${FAIL_COUNT} FAIL${RESET}"
echo -e "${BOLD}====================================================================${RESET}"

if [ "${#FAIL_ITEMS[@]}" -gt 0 ]; then
    echo -e "${RED}${BOLD}Failures requiring action:${RESET}"
    for item in "${FAIL_ITEMS[@]}"; do
        echo -e "  ${RED}✖${RESET} ${item}"
    done
    echo ""
fi

if [ "${#WARN_ITEMS[@]}" -gt 0 ]; then
    echo -e "${YELLOW}${BOLD}Warnings / Optional items:${RESET}"
    for item in "${WARN_ITEMS[@]}"; do
        echo -e "  ${YELLOW}⚠${RESET} ${item}"
    done
    echo ""
fi

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo -e "${GREEN}${BOLD}✅ Pi 5 setup looks healthy and ready to run ConeRobot!${RESET}"
    exit 0
else
    echo -e "${RED}${BOLD}❌ Fix the ${FAIL_COUNT} failure(s) above, then re-run: bash scripts/check_rpi5_setup.sh${RESET}"
    exit 1
fi
