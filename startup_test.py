"""Bare startup test: spin Motor HAT motor1 forward, then reverse.

No encoders. Run on the Pi:

    python3 startup_test.py

Press Enter at any time to stop.
"""

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

import stop_on_enter

THROTTLE = 0.4
SECONDS = 5


def main():
    stop_on_enter.install()
    kit = MotorKit(i2c=board.I2C())
    motor = kit.motor1

    try:
        print(f"Motor 1 forward for {SECONDS}s...")
        motor.throttle = THROTTLE
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Stop.")
        motor.throttle = 0
        if stop_on_enter.sleep(0.5):
            print("Stopped.")
            return

        print(f"Motor 1 reverse for {SECONDS}s...")
        motor.throttle = -THROTTLE
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Done.")
    finally:
        motor.throttle = 0


if __name__ == "__main__":
    main()
