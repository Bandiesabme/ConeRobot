# ⚡ Raspberry Pi Zero (W / Zero 2 W) Setup & Hardware Configuration Guide

This guide details the base operating system preparation, package installation, PL011 hardware UART configuration, and Wi-Fi optimization for a **Raspberry Pi Zero W** or **Raspberry Pi Zero 2 W** running **Raspberry Pi OS (Bookworm, 32-bit or 64-bit Lite)**.

This setup is required prior to running standalone field applications like the **[RTK Base Station Caster](BASE_STATION_SETUP.md)**.

---

## 1. Package Installation (Raspberry Pi OS Lite)

> [!NOTE]
> Fresh installations of **Raspberry Pi OS Lite** do **not** include `git` or `pip` by default (which triggers `-bash: git: command not found`).
> Additionally, Debian Bookworm enforces PEP 668 ("externally-managed-environment"), so Python packages should be installed via `apt` rather than global `pip`.

Connect to your Pi Zero via SSH (`ssh conerobot@conerobotBaseStation.local` or through your router/hotspot IP):

```bash
# Update package lists and install git, python3, and pyserial
sudo apt update && sudo apt install -y git python3 python3-pip python3-serial
```

---

## 2. Hardware PL011 UART Configuration (`/dev/ttyAMA0`)

On the Raspberry Pi Zero, the high-performance Broadcom PL011 hardware UART (`/dev/ttyAMA0`) is assigned by default to the onboard Bluetooth module. The 40-pin header is left with the inferior "mini-UART" (`/dev/ttyS0`), which has no hardware FIFO and is tied to CPU core clock frequency scaling—causing dropped RTCM3 differential GNSS correction bytes at 115200 baud.

Applying `dtoverlay=disable-bt` permanently assigns the rock-solid hardware PL011 UART to GPIO 14/15 (`/dev/ttyAMA0`):

### Step 2.1: Enable UART & Assign Hardware PL011 to GPIO 14/15
```bash
sudo bash -c "cat << 'EOF' >> /boot/firmware/config.txt

[all]
enable_uart=1
dtoverlay=disable-bt
EOF"
```

### Step 2.2: Disable Bluetooth Services
```bash
sudo systemctl disable hciuart.service 2>/dev/null || true
sudo systemctl disable bluetooth.service 2>/dev/null || true
```

### Step 2.3: Disable Serial Login Console
```bash
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true

CMDLINE_FILE="/boot/firmware/cmdline.txt"
[ ! -f "$CMDLINE_FILE" ] && CMDLINE_FILE="/boot/cmdline.txt"
sudo sed -i 's/console=serial0,[0-9]\+ //g; s/console=ttyAMA0,[0-9]\+ //g' "$CMDLINE_FILE"
```

### Step 2.4: Add User to Dialout Group
```bash
sudo usermod -aG dialout $USER
```

---

## 3. Wi-Fi Optimization & Hotspot Compatibility

```bash
# Disable Wi-Fi power save (eliminates high latency spikes and dropped TCP socket streams)
sudo mkdir -p /etc/NetworkManager/conf.d/
sudo tee /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf << 'EOF'
[connection]
wifi.powersave = 2
EOF
```

> [!WARNING]
> **Connecting to a Laptop Mobile Hotspot**:
> - The **Raspberry Pi Zero W** only possesses a **2.4 GHz Wi-Fi** radio (802.11 b/g/n). It cannot see 5 GHz networks!
> - If you are using Windows Mobile Hotspot, open **Settings $\rightarrow$ Network & Internet $\rightarrow$ Mobile Hotspot $\rightarrow$ Edit**, and ensure:
>   - **Network band**: `2.4 GHz` (or `Any available`)
>   - **Network properties**: `WPA2-Personal` (do not use pure WPA3)

---

## 4. Clone the ConeRobot Repository

```bash
# Create directory and clone the repository
mkdir -p ~/github
cd ~/github
git clone https://github.com/Bandiesabme/ConeRobot.git ~/github/ConeRobot
```

---

## 5. Reboot & Verify Hardware UART

```bash
sudo reboot
```

After the Pi Zero reboots and you reconnect over SSH, verify that `/dev/ttyAMA0` is present and accessible:

```bash
ls -l /dev/ttyAMA0
```
*Expected Output:*
```text
crw-rw---- 1 root dialout 204, 64 ... /dev/ttyAMA0
```

Your Raspberry Pi Zero is now fully prepared. Continue with the **[RTK Base Station Setup Guide](BASE_STATION_SETUP.md)** to test the caster and configure it to **[Auto-Start on Boot](BASE_STATION_SETUP.md#5-auto-start-on-boot-systemd-background-service)** whenever the Pi receives power.
