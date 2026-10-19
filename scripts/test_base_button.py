#!/usr/bin/env python3
"""
==============================================================================
Raspberry Pi Zero Hardware Button Diagnostic Monitor (GPIO 17 / D17)
==============================================================================
Tests the physical button wired between GPIO 17 (Physical Pin 11) and GND.
Uses internal pull-up resistor (Active-LOW).

Wiring:
  • Button Terminal 1 -> GPIO 17 / D17 (Physical Pin 11)
  • Button Terminal 2 -> GND (Physical Pin 9 or Pin 6, 14, 20)

Usage:
    python3 test_base_button.py
==============================================================================
"""

import sys
import time

BUTTON_PIN = 17

def get_button_reader():
    """Initializes and returns a callable that returns True when button is pushed."""
    # 1. Try gpiozero (Official Raspberry Pi recommendation)
    try:
        from gpiozero import Button
        btn = Button(BUTTON_PIN, pull_up=True, bounce_time=0.05)
        print("✅ Backend: gpiozero (Python library)")
        return lambda: bool(btn.is_pressed)
    except Exception:
        pass

    # 2. Try RPi.GPIO
    try:
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print("✅ Backend: RPi.GPIO")
        return lambda: GPIO.input(BUTTON_PIN) == GPIO.LOW
    except Exception:
        pass

    # 3. Try gpiod
    try:
        import gpiod
        chip = gpiod.Chip('gpiochip0')
        line = chip.get_line(BUTTON_PIN)
        line.request(consumer="conerobot-test", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        print("✅ Backend: gpiod (libgpiod)")
        return lambda: line.get_value() == 0
    except Exception:
        pass

    return None


def main():
    print("=" * 66)
    print("  🔘 RASPBERRY PI ZERO HARDWARE BUTTON TEST (GPIO 17 / D17)")
    print("=" * 66)
    print("  • Physical Pin: Pin 11 (GPIO 17 / D17)")
    print("  • Common Pin  : Pin 9 (GND)")
    print("  • Mode        : Active-LOW (Internal Pull-Up Enabled)")
    print("=" * 66 + "\n")

    reader = get_button_reader()
    if reader is None:
        print("❌ Error: No GPIO library available on this system.")
        print("Install gpiozero or RPi.GPIO on the Pi Zero:")
        print("    sudo apt update && sudo apt install -y python3-gpiozero python3-rpi.gpio")
        sys.exit(1)

    print("🚀 Monitoring button state in real-time... Press Ctrl+C to stop.\n")
    last_state = None

    try:
        while True:
            pressed = reader()
            if pressed != last_state:
                timestamp = time.strftime("%H:%M:%S")
                if pressed:
                    print(f"[{timestamp}] 🟢 PUSHED!   (GPIO 17 pulled to GND)")
                else:
                    print(f"[{timestamp}] ⚪ RELEASED  (GPIO 17 pulled UP to 3.3V)")
                last_state = pressed
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")


if __name__ == "__main__":
    main()
