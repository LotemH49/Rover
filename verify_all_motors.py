"""Drive all four motors at once to verify the rover can move.

Uses MOTOR_SIGN so mirrored right-side motors drive straight.
No encoders.

Run on the Pi:

    python3 verify_all_motors.py
"""

import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

# Same as rover.py: right side is mounted mirrored.
#   1 = front-right   2 = front-left   3 = rear-left   4 = rear-right
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}

THROTTLE = 0.4
SECONDS = 5
PAUSE = 1.0


def set_all(motors, logical):
    """Apply the same logical forward(+)/backward(-) throttle to all wheels."""
    for num, motor in motors.items():
        motor.throttle = MOTOR_SIGN[num] * logical


def main():
    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }

    try:
        print(f"Drive FORWARD {SECONDS}s (all motors)...")
        set_all(motors, +THROTTLE)
        time.sleep(SECONDS)

        print("Stop.")
        set_all(motors, 0)
        time.sleep(PAUSE)

        print(f"Drive REVERSE {SECONDS}s (all motors)...")
        set_all(motors, -THROTTLE)
        time.sleep(SECONDS)

        print("Done.")
    finally:
        set_all(motors, 0)


if __name__ == "__main__":
    main()
