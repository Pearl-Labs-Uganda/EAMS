#!/usr/bin/env python3
"""pwm_bench.py -- verify BOTH enable pins actually produce PWM on the Jetson.

Why this exists
---------------
Jetson.GPIO has an ordering quirk: if two PWM-capable pins are both put into
GPIO output mode before either GPIO.PWM() object is constructed, only the
LAST-constructed PWM channel actually drives its pin. The other sits at a
static level. On an L298N enable line that means a motor channel that is
either dead or permanently enabled at 100% duty -- the latter bypasses
DUTY_CAP, which (with no MPU6050 fitted) is our only thermal protection.

This script uses the correct interleaved pattern and reports what the kernel
thinks each channel is doing, so we confirm rather than assume.

Safety
------
IN1..IN4 are driven LOW and held there for the whole run. With both direction
inputs of an L298N channel low, that channel's outputs are off no matter what
the enable pin does -- so the motors cannot spin. Even so: run this the first
time with the motor battery DISCONNECTED, and wheels off the ground after that.

Usage
-----
    cd rccar/
    python3 pwm_bench.py            # 15 s at 20% duty on both channels

Then, in a SECOND terminal while it runs:
    sudo cat /sys/kernel/debug/pwm

Both PWM chips should be listed as enabled with a nonzero duty. If only one
is, the ordering bug is reproduced (or the pin is not muxed for PWM at all).
"""

import sys
import time
from pathlib import Path

import config

DUTY_PERCENT = 20.0       # deliberately low; we are not testing torque
HOLD_S = 15.0


def report_sysfs():
    """Best-effort read of exported PWM channels without needing sudo."""
    root = Path("/sys/class/pwm")
    if not root.exists():
        print("  (no /sys/class/pwm -- kernel PWM not present?)")
        return
    found = False
    for chip in sorted(root.glob("pwmchip*")):
        for chan in sorted(chip.glob("pwm*")):
            try:
                period = (chan / "period").read_text().strip()
                duty = (chan / "duty_cycle").read_text().strip()
                enable = (chan / "enable").read_text().strip()
            except OSError as e:
                print(f"  {chip.name}/{chan.name}: unreadable ({e})")
                continue
            found = True
            print(f"  {chip.name}/{chan.name}: enable={enable} "
                  f"period={period}ns duty={duty}ns")
    if not found:
        print("  (nothing exported yet -- try: sudo cat /sys/kernel/debug/pwm)")


def main():
    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        print("Jetson.GPIO not importable -- this script must run on the Jetson.")
        return 1

    print(f"ENA (left)  = BOARD pin {config.ENA}")
    print(f"ENB (right) = BOARD pin {config.ENB}")
    print(f"freq = {config.PWM_FREQ_HZ} Hz, duty = {DUTY_PERCENT}%\n")

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    dir_pins = (config.IN1, config.IN2, config.IN3, config.IN4)
    pwm_a = pwm_b = None

    try:
        # Direction pins low first and left alone -- H-bridge outputs off.
        for pin in dir_pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        print("Direction pins IN1-IN4 held LOW (motor outputs disabled).\n")

        # THE POINT OF THIS SCRIPT: setup immediately followed by PWM
        # construction, one channel fully done before the next is touched.
        GPIO.setup(config.ENA, GPIO.OUT, initial=GPIO.LOW)
        pwm_a = GPIO.PWM(config.ENA, config.PWM_FREQ_HZ)
        pwm_a.start(DUTY_PERCENT)
        print(f"ENA (pin {config.ENA}) started.")

        GPIO.setup(config.ENB, GPIO.OUT, initial=GPIO.LOW)
        pwm_b = GPIO.PWM(config.ENB, config.PWM_FREQ_HZ)
        pwm_b.start(DUTY_PERCENT)
        print(f"ENB (pin {config.ENB}) started.\n")

        print("Kernel view of exported PWM channels:")
        report_sysfs()

        print(f"\nHolding {HOLD_S:.0f}s. In another terminal run:")
        print("    sudo cat /sys/kernel/debug/pwm")
        print("PASS = both chips enabled with nonzero duty.")
        print("FAIL = only one has duty -> that pin is not really doing PWM.\n")

        time.sleep(HOLD_S)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for pwm in (pwm_a, pwm_b):
            if pwm is not None:
                try:
                    pwm.stop()
                except Exception as e:
                    print(f"PWM stop failed: {e}")
        for pin in dir_pins + (config.ENA, config.ENB):
            try:
                GPIO.output(pin, GPIO.LOW)
            except RuntimeError:
                pass
        try:
            GPIO.cleanup()
        except OSError as e:
            print(f"Cleanup warning (safe to ignore): {e}")
        print("Shutdown complete -- all pins low.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
