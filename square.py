"""Drive a square: forward, then spin-in-place turn, four times.

Timed open-loop (no encoders). Uses verified motor map from rover.py.

Run on the Pi:

    python3 square.py

Press Enter at any time to stop.
"""

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

import stop_on_enter

# Must match rover.py (verified: 1=RR, 2=RL, 3=FL, 4=FR)
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}
LEFT_MOTORS = (2, 3)   # rear-left, front-left
RIGHT_MOTORS = (1, 4)  # rear-right, front-right

DRIVE_THROTTLE = 0.5
DRIVE_SECONDS = 2.0
TURN_THROTTLE = 1.0
TURN_SECONDS = 0.5
SIDES = 4


def set_sides(motors, left, right):
    for num in LEFT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * left
    for num in RIGHT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * right


def stop(motors):
    set_sides(motors, 0, 0)


def main():
    stop_on_enter.install()
    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }

    try:
        for side in range(1, SIDES + 1):
            print(f"Side {side}/{SIDES}: drive forward {DRIVE_SECONDS}s @ {DRIVE_THROTTLE}")
            set_sides(motors, DRIVE_THROTTLE, DRIVE_THROTTLE)
            if stop_on_enter.sleep(DRIVE_SECONDS):
                print("Stopped.")
                return

            stop(motors)

            print(f"Side {side}/{SIDES}: turn left in place {TURN_SECONDS}s @ {TURN_THROTTLE}")
            # CCW: left backward, right forward
            set_sides(motors, -TURN_THROTTLE, +TURN_THROTTLE)
            if stop_on_enter.sleep(TURN_SECONDS):
                print("Stopped.")
                return

            stop(motors)

        print("Square done.")
    finally:
        stop(motors)


if __name__ == "__main__":
    main()
