"""WASD control — smooth hold-to-move using rover motor mapping.

W/S = forward / back (continuous while held)
A/D = spin left / right (continuous while held)
- / = = drive throttle ±0.1
[ / ] = turn throttle ±0.1
1 / 2 = hold-release timeout ±0.1s

Encoder closed-loop nudges (one shot per tap):
  e / c = drive forward / back by step_mm
  q / z = turn left / right by step_deg
  3 / 4 = step_mm ±10
  5 / 6 = step_deg ±5

Enter to quit.

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
TIMEOUT_STEP = 0.1
TIMEOUT_MIN, TIMEOUT_MAX = 0.15, 2.0
DIST_STEP = 10.0
ANGLE_STEP = 5.0
DIST_MIN, DIST_MAX = 10.0, 500.0
ANGLE_MIN, ANGLE_MAX = 5.0, 180.0


def clamp_throttle(v):
    return max(0.1, min(1.0, round(v, 1)))


def clamp_timeout(v):
    return max(TIMEOUT_MIN, min(TIMEOUT_MAX, round(v, 2)))


def clamp_dist(v):
    return max(DIST_MIN, min(DIST_MAX, round(v)))


def clamp_angle(v):
    return max(ANGLE_MIN, min(ANGLE_MAX, round(v)))


def status(drive_th, turn_th, timeout, step_mm, step_deg):
    print(
        f"  drive={drive_th:.1f}  turn={turn_th:.1f}  "
        f"timeout={timeout:.2f}s  nudge={step_mm:.0f}mm/{step_deg:.0f}deg",
        flush=True,
    )


def set_command(bot, cmd, drive_th, turn_th):
    if cmd == "w":
        bot._drive_sides(drive_th, drive_th)
    elif cmd == "s":
        bot._drive_sides(-drive_th, -drive_th)
    elif cmd == "a":
        bot._drive_sides(-turn_th, turn_th)
    elif cmd == "d":
        bot._drive_sides(turn_th, -turn_th)
    else:
        bot.stop()


def run_nudge(bot, fn):
    """Run a blocking encoder nudge; allow Enter to abort."""
    thread = threading.Thread(target=fn, daemon=True)
    thread.start()
    while thread.is_alive():
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                stop_on_enter._stop.set()
                thread.join()
                return "\n"
            # Buffer other keys for after nudge.
            return ch
        thread.join(0.05)
    return None


def main():
    drive_th = 0.5
    turn_th = 1.0
    timeout = 0.35
    step_mm = 50.0
    step_deg = 15.0

    stop_on_enter._stop = threading.Event()

    print("WASD control")
    print("  Hold W/S/A/D to move (smooth continuous).")
    print("  e/c encoder drive nudge   q/z encoder turn nudge")
    print("  -/= drive th   [/] turn th   1/2 hold timeout")
    print("  3/4 nudge mm   5/6 nudge deg   Enter quit\n")
    status(drive_th, turn_th, timeout, step_mm, step_deg)
    print()

    bot = rover_mod.Rover()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    cmd = None
    last_key = 0.0
    pending = None

    try:
        tty.setcbreak(fd)
        while not stop_on_enter.stopped():
            ch = None
            if pending is not None:
                ch = pending
                pending = None
            elif select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)

            if ch is not None:
                if ch in ("\n", "\r"):
                    break
                key = ch.lower()

                if key in "wasd":
                    cmd = key
                    last_key = time.monotonic()
                elif key == "e":
                    bot.stop()
                    cmd = None
                    result = run_nudge(
                        bot, lambda: bot.drive(step_mm, throttle=drive_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif key == "c":
                    bot.stop()
                    cmd = None
                    result = run_nudge(
                        bot, lambda: bot.drive(-step_mm, throttle=drive_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif key == "q":
                    bot.stop()
                    cmd = None
                    result = run_nudge(
                        bot, lambda: bot.turn(step_deg, throttle=turn_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif key == "z":
                    bot.stop()
                    cmd = None
                    result = run_nudge(
                        bot, lambda: bot.turn(-step_deg, throttle=turn_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif ch == "-" or key == "_":
                    drive_th = clamp_throttle(drive_th - THROTTLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch in ("=", "+"):
                    drive_th = clamp_throttle(drive_th + THROTTLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "[":
                    turn_th = clamp_throttle(turn_th - THROTTLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "]":
                    turn_th = clamp_throttle(turn_th + THROTTLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "1":
                    timeout = clamp_timeout(timeout - TIMEOUT_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "2":
                    timeout = clamp_timeout(timeout + TIMEOUT_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "3":
                    step_mm = clamp_dist(step_mm - DIST_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "4":
                    step_mm = clamp_dist(step_mm + DIST_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "5":
                    step_deg = clamp_angle(step_deg - ANGLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)
                elif ch == "6":
                    step_deg = clamp_angle(step_deg + ANGLE_STEP)
                    status(drive_th, turn_th, timeout, step_mm, step_deg)

            if cmd and (time.monotonic() - last_key) > timeout:
                cmd = None

            set_command(bot, cmd, drive_th, turn_th)
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
