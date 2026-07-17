"""WASD control using encoder closed-loop drive/turn (rover.py).

W/S = drive forward / back by step_mm (encoders)
A/D = spin left / right by step_deg (encoders)
- / = = drive throttle ±0.1
[ / ] = turn throttle ±0.1
1 / 2 = step distance ±10 mm
3 / 4 = step angle ±5 deg

Hold a key (key-repeat) to keep stepping. Enter aborts and quits.

Run on the Pi:

    python3 wasd.py
"""

import select
import sys
import termios
import threading
import time
import tty

import stop_on_enter
import rover as rover_mod

THROTTLE_STEP = 0.1
DIST_STEP = 10.0
ANGLE_STEP = 5.0
DIST_MIN, DIST_MAX = 10.0, 500.0
ANGLE_MIN, ANGLE_MAX = 5.0, 180.0


def clamp_throttle(value):
    return max(0.1, min(1.0, round(value, 1)))


def clamp_dist(value):
    return max(DIST_MIN, min(DIST_MAX, round(value)))


def clamp_angle(value):
    return max(ANGLE_MIN, min(ANGLE_MAX, round(value)))


def status(drive_th, turn_th, step_mm, step_deg):
    print(
        f"  drive_th={drive_th:.1f}  turn_th={turn_th:.1f}  "
        f"step={step_mm:.0f}mm / {step_deg:.0f}deg",
        flush=True,
    )


def run_move(bot, fn):
    """Run a blocking rover move in a thread so we can still read Enter."""
    thread = threading.Thread(target=fn, daemon=True)
    thread.start()
    while thread.is_alive():
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                stop_on_enter._stop.set()
                thread.join()
                return "\n"
            return ch
        thread.join(0.05)
    return None


def main():
    drive_th = 0.5
    turn_th = 1.0
    step_mm = 50.0
    step_deg = 15.0

    stop_on_enter._stop = threading.Event()

    print("WASD encoder control (closed-loop)")
    print("  W/S drive step   A/D turn step")
    print("  -/= drive throttle   [/] turn throttle")
    print("  1/2 step mm ±10   3/4 step deg ±5")
    print("  Hold to repeat steps. Enter to quit.\n")
    status(drive_th, turn_th, step_mm, step_deg)
    print()

    bot = rover_mod.Rover()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    pending = None

    try:
        tty.setcbreak(fd)

        while not stop_on_enter.stopped():
            if pending is not None:
                ch = pending
                pending = None
            elif select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
            else:
                continue

            if ch in ("\n", "\r"):
                break

            key = ch.lower()

            if key in "wasd":
                while not stop_on_enter.stopped():
                    if key == "w":
                        move = lambda: bot.drive(step_mm, throttle=drive_th)
                    elif key == "s":
                        move = lambda: bot.drive(-step_mm, throttle=drive_th)
                    elif key == "a":
                        move = lambda: bot.turn(step_deg, throttle=turn_th)
                    else:
                        move = lambda: bot.turn(-step_deg, throttle=turn_th)

                    result = run_move(bot, move)

                    if result == "\n" or stop_on_enter.stopped():
                        stop_on_enter._stop.set()
                        break

                    # If a key arrived mid-move, handle it next.
                    if result is not None:
                        k = result.lower()
                        if k in "wasd":
                            key = k
                            continue
                        pending = result
                        break

                    # After a finished step, see if key-repeat wants another.
                    time.sleep(0.02)
                    nxt = None
                    while select.select([sys.stdin], [], [], 0)[0]:
                        ch2 = sys.stdin.read(1)
                        if ch2 in ("\n", "\r"):
                            stop_on_enter._stop.set()
                            nxt = None
                            break
                        k2 = ch2.lower()
                        if k2 in "wasd":
                            nxt = k2
                        else:
                            pending = ch2
                    if stop_on_enter.stopped():
                        break
                    if nxt is None:
                        break
                    key = nxt
                continue

            if key == "-" or key == "_":
                drive_th = clamp_throttle(drive_th - THROTTLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key in ("=", "+"):
                drive_th = clamp_throttle(drive_th + THROTTLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "[":
                turn_th = clamp_throttle(turn_th - THROTTLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "]":
                turn_th = clamp_throttle(turn_th + THROTTLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "1":
                step_mm = clamp_dist(step_mm - DIST_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "2":
                step_mm = clamp_dist(step_mm + DIST_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "3":
                step_deg = clamp_angle(step_deg - ANGLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)
            elif key == "4":
                step_deg = clamp_angle(step_deg + ANGLE_STEP)
                status(drive_th, turn_th, step_mm, step_deg)

    finally:
        stop_on_enter._stop.set()
        bot.stop()
        try:
            bot.cleanup()
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("Quit.")


if __name__ == "__main__":
    main()
