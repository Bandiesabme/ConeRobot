#!/usr/bin/env python3
"""
Cross-Process I2C Bus Lock & Automatic Stuck-SDA Recovery for Raspberry Pi 5.

Why this exists:
  1. BNO08x SHTP reads span multiple consecutive I2C transactions (4-byte header
     then payload). Without a cross-process lock, polling INA219 (0x40) from
     system_info_node can interleave between BNO08x (0x4A) transactions and
     desync the BNO08x I2C state machine.
  2. If EMI (e.g., brushed motor PWM) or an interrupted read leaves a slave
     holding SDA LOW (GPIO2=0, GPIO3=1), Pi 5's i2c-designware logs
     'controller timed out' until cleared. This helper recovers automatically
     via an optional BNO08x RST GPIO pin and/or open-drain 16-clock SCL clear.
"""

from contextlib import contextmanager
import fcntl
import os
import shutil
import subprocess
import time
from typing import Optional, Tuple


@contextmanager
def i2c_bus_lock(bus_num: int = 1):
    """Advisory cross-process flock guarding /dev/i2c-<bus_num> transactions."""
    lock_path = f"/tmp/conerobot_i2c{int(bus_num)}.lock"
    fd = None
    try:
        try:
            fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
            try:
                os.chmod(lock_path, 0o666)
            except OSError:
                pass
            fcntl.flock(fd, fcntl.LOCK_EX)
        except OSError:
            fd = None
        yield
    finally:
        if fd is not None:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                os.close(fd)
            except OSError:
                pass


def _open_rp1_gpiochip():
    """Opens the Raspberry Pi 5 RP1 GPIO chip via lgpio (chip 4 on 6.8, chip 0 on 6.12+)."""
    import lgpio
    for chip in (4, 0):
        if not os.path.exists(f"/dev/gpiochip{chip}"):
            continue
        try:
            h = lgpio.gpiochip_open(chip)
            info = lgpio.gpio_get_chip_info(h)
            label = str(info[2]).lower() if len(info) > 2 else ""
            if "rp1" in label or chip == 0:
                return lgpio, h
            lgpio.gpiochip_close(h)
        except Exception:
            pass
    return lgpio, lgpio.gpiochip_open(0)


def read_i2c1_sda_level() -> Optional[int]:
    """Returns current logic level (0 or 1) of Pi 5 GPIO 2 (SDA1), or None if unavailable."""
    try:
        lgpio, h = _open_rp1_gpiochip()
        try:
            return int(lgpio.gpio_read(h, 2))
        finally:
            lgpio.gpiochip_close(h)
    except Exception:
        return None


def pulse_reset_pin(reset_pin: int, low_ms: int = 25, settle_ms: int = 150) -> bool:
    """Hardware-resets an active-low BNO08x RST pin connected to a Pi 5 GPIO."""
    if reset_pin is None or int(reset_pin) < 0:
        return False
    pin = int(reset_pin)
    try:
        lgpio, h = _open_rp1_gpiochip()
        try:
            lgpio.gpio_claim_output(h, pin, 1)
            lgpio.gpio_write(h, pin, 0)
            time.sleep(max(0.01, low_ms / 1000.0))
            lgpio.gpio_write(h, pin, 1)
        finally:
            try:
                lgpio.gpio_free(h, pin)
            except Exception:
                pass
            lgpio.gpiochip_close(h)
        time.sleep(max(0.05, settle_ms / 1000.0))
        return True
    except Exception:
        return False


def clear_i2c1_bus_pinctrl(max_clocks: int = 16) -> Tuple[bool, str]:
    """
    Open-drain safe I2C1 bus clear (UM10204 §3.1.16) using Raspberry Pi's `pinctrl`:
      - Toggles GPIO 3 (SCL1) between drive-LOW (`op dl`) and pull-up release (`ip pu`)
        up to `max_clocks` times until GPIO 2 (SDA1) releases to HIGH.
      - Generates a STOP condition, then ALWAYS restores GPIO 2 & 3 to `a3 pu` (I2C1).
    """
    pinctrl_bin = shutil.which("pinctrl")
    if not pinctrl_bin:
        return False, "pinctrl not installed (install with: sudo apt install -y raspi-utils)"

    def _run(*args):
        return subprocess.run(
            [pinctrl_bin, *args],
            capture_output=True,
            text=True,
            timeout=1.0,
        )

    def _sda_is_high() -> bool:
        res = _run("get", "2")
        return "| hi" in (res.stdout or "")

    try:
        # Release SDA as input with pull-up and clock SCL open-drain until SDA returns HIGH
        _run("set", "2", "ip", "pu")
        clocks_sent = 0
        for _ in range(max_clocks):
            if _sda_is_high():
                break
            _run("set", "3", "op", "dl")
            time.sleep(0.0005)
            _run("set", "3", "ip", "pu")
            time.sleep(0.0005)
            clocks_sent += 1

        # Send I2C STOP condition: SCL low -> SDA low -> SCL high -> SDA high
        _run("set", "3", "op", "dl")
        time.sleep(0.0005)
        _run("set", "2", "op", "dl")
        time.sleep(0.0005)
        _run("set", "3", "ip", "pu")
        time.sleep(0.0005)
        _run("set", "2", "ip", "pu")
        time.sleep(0.001)

        sda_ok = _sda_is_high()
        return sda_ok, f"pinctrl 9-clock clear ({clocks_sent} clocks, SDA={'HIGH' if sda_ok else 'STILL LOW'})"
    except Exception as e:
        return False, f"pinctrl recovery error: {e}"
    finally:
        # ALWAYS restore Pi 5 RP1 GPIO 2 & 3 back to hardware I2C1 alt function (a3 pu)
        try:
            _run("set", "2", "a3", "pu")
            _run("set", "3", "a3", "pu")
        except Exception:
            pass


def recover_i2c1_bus(reset_pin: int = -1) -> Tuple[bool, str]:
    """
    Attempts automatic non-reboot recovery of a stuck I2C1 bus (SDA=0):
      1. Pulses hardware RST GPIO if configured (`reset_pin >= 0`).
      2. Runs open-drain `pinctrl` SCL clock-clear if SDA is still LOW.
    """
    notes = []

    if reset_pin is not None and int(reset_pin) >= 0:
        if pulse_reset_pin(int(reset_pin)):
            sda = read_i2c1_sda_level()
            notes.append(f"RST GPIO{int(reset_pin)} pulsed (SDA={sda})")
            if sda == 1:
                return True, "; ".join(notes)
        else:
            notes.append(f"RST GPIO{int(reset_pin)} pulse failed")

    sda_before = read_i2c1_sda_level()
    if sda_before == 1:
        return True, "; ".join(notes) if notes else "SDA already HIGH"

    ok, detail = clear_i2c1_bus_pinctrl()
    notes.append(detail)
    if ok:
        return True, "; ".join(notes)

    sda_after = read_i2c1_sda_level()
    return (sda_after == 1), "; ".join(notes)
