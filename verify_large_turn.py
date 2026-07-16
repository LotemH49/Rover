"""Large-radius (arc) turn test — not spin-in-place.

Both sides drive forward; outer side is faster so the rover curves.
Uses the verified motor map from rover.py:
  M1=RR, M2=RL, M3=FL, M4=FR

Run on the Pi:

    python3 verify_large_turn.py

Press Enter at any time to stop.
"""

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

import stop_on_enter

# Must match rover.py (verified: 1=RR, 2=RL, 3=FL, 4=FR)
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}
LEFT_MOTORS = (2, 3)   # rear-left, front-left
RIGHT_MOTORS = (1, 4)  # rear-right, front-right

# Outer / inner throttle for an arc (both forward).
OUTER = 1
INNER = .2
SECONDS = 5
PAUSE = 0


def set_sides(motors, left, right):
    """Set left/right logical throttles (forward +, backward -)."""
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
        print(f"Large LEFT turn (arc) {SECONDS}s...")
        print(f"  left={INNER}  right={OUTER}")
        set_sides(motors, INNER, OUTER)
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Stop.")
        stop(motors)
        if stop_on_enter.sleep(PAUSE):
            print("Stopped.")
            return

        print(f"Large RIGHT turn (arc) {SECONDS}s...")
        print(f"  left={OUTER}  right={INNER}")
        set_sides(motors, INNER, OUTER)
        if stop_on_enter.sleep(SECONDS):
            print("Stopped.")
            return

        print("Done.")
    finally:
        stop(motors)


if __name__ == "__main__":
    main()
