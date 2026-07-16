"""Spin-in-place turn test for all four motors.

No encoders — timed open-loop spin left, then right.
Uses the same MOTOR_SIGN as rover.py / verify_all_motors.py.

Run on the Pi:

    python3 verify_turn.py
"""

import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

# Must match rover.py
#   1 = front-left   2 = front-right   3 = rear-left   4 = rear-right
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}
LEFT_MOTORS = (1, 3)
RIGHT_MOTORS = (2, 4)

THROTTLE = 0.3
SECONDS = 5
PAUSE = 1.0


def set_sides(motors, left, right):
    """Set left/right logical throttles (forward +, backward -)."""
    for num in LEFT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * left
    for num in RIGHT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * right


def stop(motors):
    set_sides(motors, 0, 0)


def main():
    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }

    try:
        # CCW / left turn: left backward, right forward
        print(f"Turn LEFT (CCW) {SECONDS}s...")
        set_sides(motors, -THROTTLE, +THROTTLE)
        time.sleep(SECONDS)

        print("Stop.")
        stop(motors)
        time.sleep(PAUSE)

        # CW / right turn: left forward, right backward
        print(f"Turn RIGHT (CW) {SECONDS}s...")
        set_sides(motors, +THROTTLE, -THROTTLE)
        time.sleep(SECONDS)

        print("Done.")
    finally:
        stop(motors)


if __name__ == "__main__":
    main()
