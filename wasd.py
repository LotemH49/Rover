"""WASD manual rover control.

W/S = forward / back
A/D = spin left / right in place
- / = = lower / raise drive throttle by 0.1
[ / ] = lower / raise turn throttle by 0.1

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

# Longer than typical keyboard initial-repeat delay so hold doesn't stutter.
RELEASE_TIMEOUT = 0.7
STEP = 0.1


def clamp(value):
    return max(0.0, min(1.0, round(value, 1)))


def set_sides(motors, left, right):
    for num in LEFT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * left
    for num in RIGHT_MOTORS:
        motors[num].throttle = MOTOR_SIGN[num] * right


def stop(motors):
    set_sides(motors, 0, 0)


def apply_command(motors, cmd, drive, turn):
    if cmd == "w":
        set_sides(motors, drive, drive)
    elif cmd == "s":
        set_sides(motors, -drive, -drive)
    elif cmd == "a":
        set_sides(motors, -turn, turn)
    elif cmd == "d":
        set_sides(motors, turn, -turn)
    else:
        stop(motors)


def main():
    drive = 1.0
    turn = 1.0

    print("WASD rover control")
    print("  W/S drive   A/D turn")
    print("  -/= drive throttle ±0.1   [/] turn throttle ±0.1")
    print("  Hold key to move, release to stop. Enter to quit.")
    print(f"  drive={drive:.1f}  turn={turn:.1f}\n")

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
                elif ch == "-" or key == "_":
                    drive = clamp(drive - STEP)
                    print(f"  drive={drive:.1f}  turn={turn:.1f}", flush=True)
                elif ch in ("=", "+"):
                    drive = clamp(drive + STEP)
                    print(f"  drive={drive:.1f}  turn={turn:.1f}", flush=True)
                elif ch == "[":
                    turn = clamp(turn - STEP)
                    print(f"  drive={drive:.1f}  turn={turn:.1f}", flush=True)
                elif ch == "]":
                    turn = clamp(turn + STEP)
                    print(f"  drive={drive:.1f}  turn={turn:.1f}", flush=True)

            if cmd and (time.monotonic() - last_key) > RELEASE_TIMEOUT:
                cmd = None

            apply_command(motors, cmd, drive, turn)
    finally:
        stop(motors)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("Quit.")


if __name__ == "__main__":
    main()
