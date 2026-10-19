#!/usr/bin/env python3
"""
==============================================================================
Raspberry Pi Zero INA219 Battery & Current Diagnostic Monitor
==============================================================================
Reads Voltage (V), Current (mA), and Power (W) when the INA219 sensor is
connected in series with the Base Station battery/power source.

Wiring in Series:
  • VCC  -> Pin 1 (3.3V)
  • GND  -> Pin 9 (Common Ground)
  • SDA  -> Pin 3 (GPIO 2)
  • SCL  -> Pin 5 (GPIO 3)
  • VIN+ -> Battery Positive (+)
  • VIN- -> Pi Zero / Buck converter 5V (+) input
  • Battery (-) & Pi Zero GND -> Common Ground

Usage:
    python3 test_base_battery.py
    python3 test_base_battery.py --loop 1.0
==============================================================================
"""

import argparse
import sys
import time


def read_ina219_data(bus_id: int = 1, address: int = 0x40):
    """
    Reads bus voltage, shunt current, and calculates power from INA219.
    Register 0x02: Bus Voltage (LSB = 4 mV, bits [15:3])
    Register 0x01: Shunt Voltage (signed 16-bit, LSB = 10 µV = 0.01 mV)
    With 0.1 Ω shunt:
      Current (mA) = Shunt_mV / 0.1 Ω = raw_shunt * 0.01 / 0.1 = raw_shunt * 0.1 mA
    """
    try:
        try:
            import smbus
        except ImportError:
            import smbus2 as smbus  # type: ignore

        bus = smbus.SMBus(bus_id)
    except Exception as e:
        return None, f"Could not open I2C bus {bus_id}: {e}. Ensure I2C is enabled ('sudo raspi-config nonint do_i2c 0') and python3-smbus is installed."

    try:
        # 1. Bus Voltage (Register 0x02)
        raw_v = bus.read_word_data(address, 0x02)
        raw_v_swapped = ((raw_v & 0xFF) << 8) | (raw_v >> 8)
        voltage = ((raw_v_swapped >> 3) & 0x1FFF) * 0.004

        # 2. Shunt Current (Register 0x01)
        raw_s = bus.read_word_data(address, 0x01)
        raw_s_swapped = ((raw_s & 0xFF) << 8) | (raw_s >> 8)
        if raw_s_swapped >= 32768:
            raw_s_swapped -= 65536

        # 10 µV per LSB -> 0.01 mV. Current = (raw * 0.01 mV) / 0.1 Ω = raw * 0.1 mA
        current_ma = raw_s_swapped * 0.1
        power_w = voltage * (abs(current_ma) / 1000.0)

        bus.close()
        return {
            "voltage": round(voltage, 2),
            "current_ma": round(current_ma, 1),
            "power_w": round(power_w, 2),
        }, None
    except Exception as e:
        bus.close()
        return None, f"I2C read failed at address 0x{address:02X}: {e}"


def get_charge_estimate(voltage: float, current_ma: float = 0.0) -> str:
    """Estimates battery percentage and runtime based on detected cell voltage."""
    if voltage >= 9.0:
        min_v, max_v = 10.5, 12.6
        pct = max(0, min(100, int((voltage - min_v) / (max_v - min_v) * 100)))
        bars = int(pct / 10)
        return f"3S LiPo ~{pct}% [{'█' * bars}{'░' * (10 - bars)}]"
    elif voltage >= 6.0:
        min_v, max_v = 7.0, 8.4
        pct = max(0, min(100, int((voltage - min_v) / (max_v - min_v) * 100)))
        bars = int(pct / 10)
        return f"2S LiPo ~{pct}% [{'█' * bars}{'░' * (10 - bars)}]"
    elif voltage >= 4.4:
        return f"5V Rail ({voltage:.2f}V)"
    elif voltage >= 2.8:
        # 1S 18650 Li-ion cell (Samsung 35E: 3.0V cutoff to 4.2V full, 3500 mAh)
        # Dynamic loop IR drop compensation (Shunt 0.10 ohm + cell IR ~0.06 ohm + contacts/wiring ~0.30 ohm = 0.457 ohm)
        v_ocv = max(3.0, min(4.20, voltage + (current_ma / 1000.0) * 0.457))

        # Empirical Samsung 35E resting OCV discharge curve
        if v_ocv >= 4.15: pct = int(95 + (v_ocv - 4.15) * 100)
        elif v_ocv >= 4.05: pct = int(85 + (v_ocv - 4.05) * 100)
        elif v_ocv >= 3.95: pct = int(75 + (v_ocv - 3.95) * 100)
        elif v_ocv >= 3.85: pct = int(62 + (v_ocv - 3.85) * 130)
        elif v_ocv >= 3.75: pct = int(45 + (v_ocv - 3.75) * 170)
        elif v_ocv >= 3.65: pct = int(28 + (v_ocv - 3.65) * 170)
        elif v_ocv >= 3.50: pct = int(12 + (v_ocv - 3.50) * 106)
        else: pct = int(max(0, (v_ocv - 3.0) / 0.50 * 12))
        pct = max(0, min(100, pct))
        bars = int(pct / 10)

        if current_ma < -15.0:
            charge_ma = abs(current_ma)
            if voltage >= 4.18 and charge_ma < 250:
                stage = "Topping Off / Full (>95%)"
            elif voltage >= 4.15:
                stage = "CV Saturation Phase (~85-95%)"
            else:
                stage = f"CC Fast Charge (~{pct}%)"
            return f"⚡ CHARGING (+{charge_ma:.0f} mA) | {stage} | OCV: {v_ocv:.2f}V"
        elif current_ma > 15.0:
            remaining_mah = 3500.0 * (pct / 100.0)
            hours_left = remaining_mah / current_ma
            return f"1S 18650 ~{pct}% [{'█' * bars}{'░' * (10 - bars)}] | Est: ~{hours_left:.1f}h left | OCV: {v_ocv:.2f}V"
        else:
            return f"1S 18650 ~{pct}% [{'█' * bars}{'░' * (10 - bars)}] (Standby / Full) | OCV: {v_ocv:.2f}V"
    else:
        return f"Cutoff / Low ({voltage:.2f}V)"


def main():
    parser = argparse.ArgumentParser(description="Read INA219 Voltage, Current, and Power on Pi Zero")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x40, help="I2C address (default: 0x40)")
    parser.add_argument("--loop", type=float, default=0.0, help="Loop interval in seconds (0 = run once)")
    args = parser.parse_args()

    print("=" * 68)
    print("  🔋 RASPBERRY PI ZERO INA219 BATTERY & CURRENT MONITOR")
    print("=" * 68)
    print(f"  • I2C Bus     : /dev/i2c-{args.bus}")
    print(f"  • Address     : 0x{args.address:02X}")
    print(f"  • Mode        : Series Sensing (Voltage + Current + Power)")
    print("=" * 68 + "\n")

    from collections import deque
    v_queue = deque(maxlen=5)
    ma_queue = deque(maxlen=5)

    while True:
        data, err = read_ina219_data(args.bus, args.address)
        timestamp = time.strftime("%H:%M:%S")

        if err:
            print(f"[{timestamp}] ❌ Error: {err}")
            print("\nTroubleshooting tips:")
            print("  1. Check I2C is enabled: 'sudo raspi-config nonint do_i2c 0'")
            print("  2. Verify wiring: VCC->Pin 1 (3.3V), GND->Pin 9, SDA->Pin 3, SCL->Pin 5")
            print("  3. Scan I2C bus: 'sudo i2cdetect -y 1' (should display 0x40)")
            print("  4. Series check: Battery (+) -> VIN+, VIN- -> Pi Zero/Buck (+)")
            print("  5. Negative check: Battery (-) & Pi Zero GND -> Common Ground")
            if args.loop <= 0:
                sys.exit(1)
        else:
            v_raw = data["voltage"]
            ma_raw = data["current_ma"]

            # Reset history if a sharp step change occurs (plug/unplug)
            if ma_queue:
                last_ma = sum(ma_queue) / len(ma_queue)
                if (last_ma < -100 and ma_raw > 50) or (last_ma > 50 and ma_raw < -100):
                    v_queue.clear()
                    ma_queue.clear()

            v_queue.append(v_raw)
            ma_queue.append(ma_raw)

            v = sum(v_queue) / len(v_queue)
            ma = sum(ma_queue) / len(ma_queue)
            w = v * (abs(ma) / 1000.0)

            estimate = get_charge_estimate(v, ma)
            print(f"[{timestamp}] ⚡ {v:5.2f} V  |  📥 {ma:6.1f} mA  |  🔥 {w:4.2f} W  |  {estimate}")

        if args.loop <= 0:
            break
        time.sleep(args.loop)


if __name__ == "__main__":
    main()
