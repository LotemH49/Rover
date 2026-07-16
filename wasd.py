"""WASD manual rover control.

W/S = forward / back @ 0.5 throttle
A/D = spin left / right in place @ 1.0 throttle

Hold a key to move; release to stop. Enter to quit.

Run on the Pi (real terminal / SSH):

    python3 wasd.py
"""

import select
import sys
import termios
import time
import tty

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

# Must match rover.py (verified: 1=RR, 2=RL, 3=FL, 4=FR)
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}
LEFT_MOTORS = (2, 3)   # rear-left, front-left
RIGHT_MOTORS = (1, 4)  # rear-right, front-right

DRIVE_THROTTLE = 0.5
TURN_THROTTLE = 1.0
# Stop shortly after key release (terminal key-repeat keeps cmd alive while held).
RELEASE_TIMEOUT = 0.12


def set_sides(motors, left, right):
    for num in LEFT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * left
    for num in RIGHT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * right


def stop(motors):
    set_sides(motors, 0, 0)


def apply_command(motors, cmd):
    if cmd == "w":
        set_sides(motors, DRIVE_THROTTLE, DRIVE_THROTTLE)
    elif cmd == "s":
        set_sides(motors, -DRIVE_THROTTLE, -DRIVE_THROTTLE)
    elif cmd == "a":
        set_sides(motors, -TURN_THROTTLE, TURN_THROTTLE)
    elif cmd == "d":
        set_sides(motors, TURN_THROTTLE, -TURN_THROTTLE)
    else:
        stop(motors)


def main():
    print("WASD rover control")
    print(f"  W/S drive @ {DRIVE_THROTTLE}   A/D turn @ {TURN_THROTTLE}")
    print("  Hold key to move, release to stop. Enter to quit.\n")

    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    cmd = None
    last_key = 0.0

    try:
        tty.setcbreak(fd)
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch in ("\n", "\r"):
                    break
                key = ch.lower()
                if key in "wasd":
                    cmd = key
                    last_key = time.monotonic()

            if cmd and (time.monotonic() - last_key) > RELEASE_TIMEOUT:
                cmd = None

            apply_command(motors, cmd)
    finally:
        stop(motors)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("Quit.")


if __name__ == "__main__":
    main()
