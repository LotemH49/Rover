"""Drive all four motors at once to verify the rover can move.

Uses MOTOR_SIGN so mirrored right-side motors drive straight.
No encoders.

Run on the Pi:

    python3 verify_all_motors.py

Press Enter at any time to stop.
"""

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

import stop_on_enter

# Same as rover.py (verified: 1=RR, 2=RL, 3=FL, 4=FR)
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}

THROTTLE = 0.4
SECONDS = 5
PAUSE = 1.0


def set_all(motors, logical):
    """Apply the same logical forward(+)/backward(-) throttle to all wheels."""
    for num, motor in motors.items():
        motor.throttle = MOTOR_SIGN[num] * logical


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
        print(f"Drive FORWARD {SECONDS}s (all motors)...")
        set_all(motors, +THROTTLE)
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Stop.")
        set_all(motors, 0)
        if stop_on_enter.sleep(PAUSE):
            print("Stopped.")
            return

        print(f"Drive REVERSE {SECONDS}s (all motors)...")
        set_all(motors, -THROTTLE)
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Done.")
    finally:
        set_all(motors, 0)


if __name__ == "__main__":
    main()
