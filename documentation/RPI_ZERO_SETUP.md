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

## 5. Power-Loss Protection (Hard Power Cuts)

The base station is often powered off by simply cutting power (about twice a day). These steps protect the SD card and the saved base coordinates. **They add no meaningful power consumption** (well under 1 mA; several actually reduce SD activity).

> [!NOTE]
> `base_station_caster.py` already saves `base_station_fixed_coords.json` and `base_station_locations.json` **atomically** (temp file + `fsync` + atomic rename), so a power cut mid-save cannot corrupt your locked coordinates. Run `git pull` in `~/github/ConeRobot` to get this version.

### Step 5.1: Keep System Logs in RAM
```bash
sudo mkdir -p /etc/systemd/journald.conf.d
sudo tee /etc/systemd/journald.conf.d/volatile.conf << 'EOF'
[Journal]
Storage=volatile
RuntimeMaxUse=20M
EOF
sudo systemctl restart systemd-journald
```

### Step 5.2: Disable Disk-Based Swap (if present)
```bash
swapon --show
```
- **No output**, or only `/dev/zram0` (RAM-based): nothing to do.
- **A swap file on the SD card** (e.g. `/var/swap`): run `sudo swapoff -a` and comment out any swap line in `/etc/fstab`. On older images you can also run `sudo systemctl disable --now dphys-swapfile`.

> [!NOTE]
> Newer Raspberry Pi OS images may not have `dphys-swapfile` at all (`Unit dphys-swapfile.service does not exist`). That is fine.

### Step 5.3: Reduce SD Writes (`noatime`, `commit=60`, tmpfs `/tmp`)
```bash
# Backup, then add commit=60 (and noatime if missing) to the root (/) ext4 entry
sudo cp /etc/fstab /etc/fstab.bak
sudo sed -i -E '/^\S+\s+\/\s+ext4\s/ { /noatime/! s/defaults/defaults,noatime/ }' /etc/fstab
sudo sed -i -E '/^\S+\s+\/\s+ext4\s/ { /commit=/! s/defaults,noatime/defaults,noatime,commit=60/ }' /etc/fstab

# Add tmpfs for /tmp (only if not already present)
grep -q '^tmpfs /tmp ' /etc/fstab || echo 'tmpfs /tmp tmpfs defaults,noatime,nosuid,size=32m 0 0' | sudo tee -a /etc/fstab

# Reload, verify (root line must contain noatime,commit=60) and test for errors
sudo systemctl daemon-reload
grep ' / ' /etc/fstab
sudo findmnt --verify
```
> [!NOTE]
> If your `/` line uses a different option than `defaults` (e.g. `PARTUUID=... / ext4 rw,...`), the `sed` won't change it. In that case edit manually with `sudo nano /etc/fstab`. To undo: `sudo cp /etc/fstab.bak /etc/fstab`.


### Step 5.4: Auto-Repair the Filesystem at Boot
```bash
# Run only ONCE (appends to the single line in cmdline.txt)
sudo sed -i '1 s/$/ fsck.repair=yes/' /boot/firmware/cmdline.txt
```

### Step 5.5: Enable the Hardware Watchdog
```bash
echo "dtparam=watchdog=on" | sudo tee -a /boot/firmware/config.txt
sudo sed -i 's/^#\?RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf
```
A hung Pi now reboots itself automatically.

### Step 5.6 (Optional): Lower Power Consumption
```bash
# Disable unused onboard audio (small saving)
grep -q '^dtparam=audio=off' /boot/firmware/config.txt || echo "dtparam=audio=off" | sudo tee -a /boot/firmware/config.txt

# Headless base station: disable HDMI output (about 20-25 mA saving)
grep -q '^hdmi_blanking=1' /boot/firmware/config.txt || echo "hdmi_blanking=1" | sudo tee -a /boot/firmware/config.txt
```
> [!NOTE]
> Keep the green ACT LED enabled: the end-of-shutdown blink pattern is used in the safe power-down procedure.

> [!TIP]
> A hard power cut is now safe, but the **3-second button hold** safe shutdown is still the best way to power down when possible.

---

## 6. Reboot & Verify Hardware UART

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

Verify the power-loss protection settings:

```bash
findmnt /      # options should include noatime
free -h        # Swap should show 0B
```

Your Raspberry Pi Zero is now fully prepared. Continue with the **[RTK Base Station Setup Guide](BASE_STATION_SETUP.md)** to test the caster and configure it to **[Auto-Start on Boot](BASE_STATION_SETUP.md#5-auto-start-on-boot-systemd-background-service)** whenever the Pi receives power.
