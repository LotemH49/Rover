"""Verify all four Motor HAT channels (M1–M4).

Spins each motor forward, then reverse. No encoders.

Run on the Pi:

    python3 verify_all_motors.py
"""

import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

THROTTLE = 0.4
SECONDS = 3
PAUSE = 0.5


def main():
    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }

    try:
        for num, motor in motors.items():
            print(f"--- Motor {num} (M{num}) ---")

            print(f"  Forward {SECONDS}s...")
            motor.throttle = THROTTLE
            time.sleep(SECONDS)

            motor.throttle = 0
            time.sleep(PAUSE)

            print(f"  Reverse {SECONDS}s...")
            motor.throttle = -THROTTLE
            time.sleep(SECONDS)

            motor.throttle = 0
            time.sleep(PAUSE)
            print(f"  Motor {num} done.\n")

        print("All motors tested.")
    finally:
        for motor in motors.values():
            motor.throttle = 0


if __name__ == "__main__":
    main()
