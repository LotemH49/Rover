#!/usr/bin/env python3
"""WASD control with arc turns (no spin-in-place).

Hold keys (terminal autorepeat); chords within the hold-timeout count as
combined:

  W       = forward
  S       = back
  W+A     = arc left  (left/inner slower)
  W+D     = arc right
  S+A/S+D = reverse arcs
  A / D   = pivot on inside wheels (outside drives, inside stopped)

- / = = drive throttle ±0.1
[ / ] = arc inner ratio ±0.1  (0.0 = pivot, 1.0 = straight)
1 / 2 = hold-release timeout ±0.1s

Encoder nudge: e / c = drive forward / back by step_mm
3 / 4 = step_mm ±10

Enter to quit.

Run on the Pi:

    python3 wasd_arc.py
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
ARC_STEP = 0.1
ARC_MIN, ARC_MAX = 0.0, 0.9
DIST_STEP = 10.0
DIST_MIN, DIST_MAX = 10.0, 500.0


def clamp_throttle(v):
    return max(0.1, min(1.0, round(v, 1)))


def clamp_timeout(v):
    return max(TIMEOUT_MIN, min(TIMEOUT_MAX, round(v, 2)))


def clamp_arc(v):
    return max(ARC_MIN, min(ARC_MAX, round(v, 1)))


def clamp_dist(v):
    return max(DIST_MIN, min(DIST_MAX, round(v)))


def status(drive_th, arc_inner, timeout, step_mm):
    print(
        f"  drive={drive_th:.1f}  arc_inner={arc_inner:.1f}  "
        f"timeout={timeout:.2f}s  nudge={step_mm:.0f}mm",
        flush=True,
    )


def sides_from_keys(keys, drive_th, arc_inner):
    """Return (left, right) throttles, or None to stop."""
    forward = "w" in keys and "s" not in keys
    backward = "s" in keys and "w" not in keys
    left = "a" in keys and "d" not in keys
    right = "d" in keys and "a" not in keys

    if forward:
        base = drive_th
    elif backward:
        base = -drive_th
    else:
        base = 0.0

    if base != 0.0:
        if left:
            return (base * arc_inner, base)
        if right:
            return (base, base * arc_inner)
        return (base, base)

    # A/D alone: pivot on inside (no counter-rotate scrub).
    if left:
        return (0.0, drive_th)
    if right:
        return (drive_th, 0.0)
    return None


def apply_sides(bot, sides):
    if sides is None:
        bot.stop()
    else:
        bot._drive_sides(sides[0], sides[1])


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
            return ch
        thread.join(0.05)
    return None


def main():
    drive_th = 0.5
    arc_inner = 0.4
    timeout = 0.40
    step_mm = 50.0

    stop_on_enter._stop = threading.Event()

    print("WASD arc control (no spin-in-place)")
    print("  Hold W forward, S back.")
    print("  Hold W+A arc left, W+D arc right (S+A / S+D reverse).")
    print("  A or D alone = pivot on inside wheels.")
    print("  Tip: keep both chord keys repeating within timeout.")
    print("  -/= drive th   [/] arc inner   1/2 hold timeout")
    print("  e/c encoder drive nudge   3/4 nudge mm   Enter quit\n")
    status(drive_th, arc_inner, timeout, step_mm)
    print()

    bot = rover_mod.Rover()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    # key -> last time seen (for chord detection via autorepeat)
    last_seen: dict[str, float] = {}
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

            now = time.monotonic()

            if ch is not None:
                if ch in ("\n", "\r"):
                    break
                key = ch.lower()

                if key in "wasd":
                    last_seen[key] = now
                elif key == "e":
                    bot.stop()
                    last_seen.clear()
                    result = run_nudge(
                        bot, lambda: bot.drive(step_mm, throttle=drive_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif key == "c":
                    bot.stop()
                    last_seen.clear()
                    result = run_nudge(
                        bot, lambda: bot.drive(-step_mm, throttle=drive_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif ch == "-" or key == "_":
                    drive_th = clamp_throttle(drive_th - THROTTLE_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch in ("=", "+"):
                    drive_th = clamp_throttle(drive_th + THROTTLE_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "[":
                    arc_inner = clamp_arc(arc_inner - ARC_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "]":
                    arc_inner = clamp_arc(arc_inner + ARC_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "1":
                    timeout = clamp_timeout(timeout - TIMEOUT_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "2":
                    timeout = clamp_timeout(timeout + TIMEOUT_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "3":
                    step_mm = clamp_dist(step_mm - DIST_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "4":
                    step_mm = clamp_dist(step_mm + DIST_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)

            active = {k for k, t in last_seen.items() if (now - t) <= timeout}
            for k in list(last_seen):
                if k not in active:
                    del last_seen[k]

            apply_sides(bot, sides_from_keys(active, drive_th, arc_inner))
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
