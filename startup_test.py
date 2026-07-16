"""Bare startup test: spin Motor HAT motor1 forward, then reverse.

No encoders. Run on the Pi:

    python3 startup_test.py
"""

import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

THROTTLE = 0.4
SECONDS = 5


def main():
    kit = MotorKit(i2c=board.I2C())
    motor = kit.motor1

    try:
        print(f"Motor 1 forward for {SECONDS}s...")
        motor.throttle = THROTTLE
        time.sleep(SECONDS)

        print("Stop.")
        motor.throttle = 0
        time.sleep(0.5)

        print(f"Motor 1 reverse for {SECONDS}s...")
        motor.throttle = -THROTTLE
        time.sleep(SECONDS)

        print("Done.")
    finally:
        motor.throttle = 0


if __name__ == "__main__":
    main()
