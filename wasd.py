"""WASD manual rover control.

W/S = forward / back
A/D = spin left / right in place
- / = = lower / raise drive throttle by 0.1
[ / ] = lower / raise turn throttle by 0.1
1 / 2 = lower / raise hold timeout by 0.1s

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

STEP = 0.1
TIMEOUT_STEP = 0.1
TIMEOUT_MIN = 0.1
TIMEOUT_MAX = 2.0


def clamp(value):
    return max(0.0, min(1.0, round(value, 1)))


def clamp_timeout(value):
    return max(TIMEOUT_MIN, min(TIMEOUT_MAX, round(value, 1)))


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


def status(drive, turn, timeout):
    print(
        f"  drive={drive:.1f}  turn={turn:.1f}  timeout={timeout:.1f}s",
        flush=True,
    )


def main():
    drive = 1.0
    turn = 1.0
    timeout = 0.5

    print("WASD rover control")
    print("  W/S drive   A/D turn")
    print("  -/= drive ±0.1   [/] turn ±0.1   1/2 timeout ±0.1s")
    print("  Hold key to move, release to stop. Enter to quit.")
    status(drive, turn, timeout)
    print()

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
                    status(drive, turn, timeout)
                elif ch in ("=", "+"):
                    drive = clamp(drive + STEP)
                    status(drive, turn, timeout)
                elif ch == "[":
                    turn = clamp(turn - STEP)
                    status(drive, turn, timeout)
                elif ch == "]":
                    turn = clamp(turn + STEP)
                    status(drive, turn, timeout)
                elif ch == "1":
                    timeout = clamp_timeout(timeout - TIMEOUT_STEP)
                    status(drive, turn, timeout)
                elif ch == "2":
                    timeout = clamp_timeout(timeout + TIMEOUT_STEP)
                    status(drive, turn, timeout)

            if cmd and (time.monotonic() - last_key) > timeout:
                cmd = None

            apply_command(motors, cmd, drive, turn)
    finally:
        stop(motors)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("Quit.")


if __name__ == "__main__":
    main()
