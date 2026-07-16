"""Identify which physical wheel is on each Motor HAT channel.

Runs M1, then M2, then M3, then M4 one at a time. Watch which wheel
moves and report back so we can fix the software mapping.

No encoders. Run on the Pi:

    python3 verify_motor_location.py
"""

import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

THROTTLE = 0.4
SECONDS = 3
PAUSE = 2.0


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
            print(f"=== Motor {num} (HAT M{num}) spinning {SECONDS}s ===")
            print("    Watch which wheel moves, then note it.")
            motor.throttle = THROTTLE
            time.sleep(SECONDS)
            motor.throttle = 0
            print(f"=== Motor {num} stopped. Pause {PAUSE}s ===\n")
            time.sleep(PAUSE)

        print("Done. Tell me which physical wheel moved for M1, M2, M3, M4")
        print("(e.g. front-left, front-right, rear-left, rear-right).")
    finally:
        for motor in motors.values():
            motor.throttle = 0


if __name__ == "__main__":
    main()
