#!/usr/bin/env python3
"""
I2C Bus Lockup Recovery Tool (NXP I2C Bus Clear Specification)
--------------------------------------------------------------
When an I2C slave is interrupted while transmitting a byte with a '0' bit,
it holds SDA low indefinitely while waiting for SCL clocks.
This script generates 9 clock pulses on SCL (GPIO 3) followed by a STOP
condition to force the stuck slave to release SDA.
"""

import sys
import time

print("=" * 60)
print("  🛠️ CONE ROBOT — I2C BUS RECOVERY & UNSTICK TOOL")
print("=" * 60)

try:
    import gpiod
    from gpiod.line import Direction, Value
    HAS_GPIOD = True
except ImportError:
    HAS_GPIOD = False

try:
    import lgpio
    HAS_LGPIO = True
except ImportError:
    HAS_LGPIO = False

if not HAS_GPIOD and not HAS_LGPIO:
    print("❌ Neither gpiod nor lgpio library found.")
    print("   Run: sudo apt install -y python3-libgpiod python3-lgpio")
    sys.exit(1)


def recover_with_lgpio():
    print("Attempting bus recovery via lgpio...")
    # On Raspberry Pi 5, RP1 40-pin header is gpiochip4
    # On Pi 4 / Pi Zero, it is gpiochip0
    for chip_num in [4, 0]:
        try:
            h = lgpio.gpiochip_open(chip_num)
        except Exception:
            continue

        try:
            # Read initial state
            sda_initial = lgpio.gpio_read(h, 2)
            scl_initial = lgpio.gpio_read(h, 3)
            print(f"  Initial State on gpiochip{chip_num}: SDA={sda_initial}, SCL={scl_initial}")

            if sda_initial == 1 and scl_initial == 1:
                print("  ✓ Bus is already free (both SDA and SCL are HIGH).")
                lgpio.gpiochip_close(h)
                return True

            print("  Generating 9 SCL clock pulses to flush slave shift register...")
            # Claim SCL as output, initially high
            lgpio.gpio_claim_output(h, 3, 1)

            for i in range(1, 10):
                lgpio.gpio_write(h, 3, 0)
                time.sleep(0.0001)
                lgpio.gpio_write(h, 3, 1)
                time.sleep(0.0001)

            # Generate STOP condition: SDA low -> high while SCL is high
            lgpio.gpio_claim_output(h, 2, 0)
            time.sleep(0.0001)
            lgpio.gpio_write(h, 3, 1)
            time.sleep(0.0001)
            lgpio.gpio_write(h, 2, 1)
            time.sleep(0.0001)

            # Release lines back to input so I2C peripheral can use them
            lgpio.gpio_claim_input(h, 2)
            lgpio.gpio_claim_input(h, 3)

            sda_final = lgpio.gpio_read(h, 2)
            scl_final = lgpio.gpio_read(h, 3)
            lgpio.gpiochip_close(h)

            print(f"  Final State: SDA={sda_final}, SCL={scl_final}")
            if sda_final == 1 and scl_final == 1:
                print("  🎉 SUCCESS! I2C Bus cleared! Slave has released SDA.")
                return True
            else:
                print(f"  ⚠️ SDA is still {sda_final}. May be a physical hardware short to GND.")
                return False
        except Exception as e:
            lgpio.gpiochip_close(h)
            print(f"  lgpio on chip {chip_num} error: {e}")

    return False


def recover_with_gpiod():
    print("Attempting bus recovery via gpiod...")
    import os
    for dev in sorted(os.listdir('/dev')):
        if not dev.startswith('gpiochip'):
            continue
        chip_path = f"/dev/{dev}"
        try:
            chip = gpiod.Chip(chip_path)
            # Find offsets for GPIO2 and GPIO3
            offset_sda = None
            offset_scl = None
            for offset in range(chip.get_info().num_lines):
                info = chip.get_line_info(offset)
                if info.name == 'GPIO2':
                    offset_sda = offset
                elif info.name == 'GPIO3':
                    offset_scl = offset

            if offset_sda is None or offset_scl is None:
                chip.close()
                continue

            print(f"  Found GPIO2/GPIO3 on {chip_path} (SDA={offset_sda}, SCL={offset_scl})")
            
            # Configure request for SCL and SDA
            line_settings = {
                offset_scl: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE),
                offset_sda: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE),
            }
            
            with chip.request_lines(consumer="i2c_recover", config=line_settings) as req:
                # 9 clock pulses on SCL
                for _ in range(9):
                    req.set_value(offset_scl, Value.INACTIVE)
                    time.sleep(0.0001)
                    req.set_value(offset_scl, Value.ACTIVE)
                    time.sleep(0.0001)

                # Send STOP condition
                req.set_value(offset_sda, Value.INACTIVE)
                time.sleep(0.0001)
                req.set_value(offset_scl, Value.ACTIVE)
                time.sleep(0.0001)
                req.set_value(offset_sda, Value.ACTIVE)
                time.sleep(0.0001)

            chip.close()
            print("  Clock pulses and STOP condition generated via gpiod.")
            return True
        except Exception as e:
            print(f"  gpiod on {chip_path} note: {e}")

    return False


success = False
if HAS_LGPIO:
    try:
        success = recover_with_lgpio()
    except Exception as e:
        print(f"lgpio failed: {e}")

if not success and HAS_GPIOD:
    try:
        success = recover_with_gpiod()
    except Exception as e:
        print(f"gpiod failed: {e}")

print("=" * 60)
if success:
    print("Run: sudo i2cdetect -y 1 to verify!")
else:
    print("If SDA remains 0, power-cycle the board completely (unplug LiPo 10s).")
