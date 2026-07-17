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

On quit, writes wasd_arc_log.txt (paste-ready stall/command summary).

Enter to quit.

Run on the Pi:

    python3 wasd_arc.py
    cat wasd_arc_log.txt
"""

from __future__ import annotations

import select
import statistics
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import stop_on_enter
import rover as rover_mod

THROTTLE_STEP = 0.1
TIMEOUT_STEP = 0.1
TIMEOUT_MIN, TIMEOUT_MAX = 0.15, 2.0
ARC_STEP = 0.1
ARC_MIN, ARC_MAX = 0.0, 0.9
DIST_STEP = 10.0
DIST_MIN, DIST_MAX = 10.0, 500.0

SAMPLE_DT = 0.05
STALL_CPS = 25.0
CMD_EPS = 0.05  # |throttle| above this counts as "commanded"
LOG_PATH = Path(__file__).resolve().parent / "wasd_arc_log.txt"


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

    if left:
        return (0.0, drive_th)
    if right:
        return (drive_th, 0.0)
    return None


def cmd_label(keys, sides) -> str:
    if sides is None:
        return "stop"
    keys_s = "".join(sorted(keys)) if keys else "-"
    return f"keys={keys_s} L={sides[0]:+.2f} R={sides[1]:+.2f}"


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


class SessionLog:
    """Record commands + encoder stall samples for paste-back."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.events: list[str] = []
        self.stall_events: list[str] = []
        self.samples = 0
        self.moving_samples = 0
        self.stall_samples = 0
        self.rates_L: list[float] = []
        self.rates_R: list[float] = []
        self._last_cmd = "stop"
        self._prev_counts: dict[int, int] | None = None
        self._prev_t: float | None = None
        self._last_sample_t = 0.0
        self._last_stall_log_t = -1.0
        self.drive_th = 0.5
        self.arc_inner = 0.4

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def note_cmd(self, label: str) -> None:
        if label == self._last_cmd:
            return
        self._last_cmd = label
        self.events.append(f"t={self.elapsed():6.2f}s  CMD  {label}")

    def sample(self, bot, sides) -> None:
        now = time.monotonic()
        if now - self._last_sample_t < SAMPLE_DT:
            return
        self._last_sample_t = now
        self.samples += 1

        counts = dict(bot.counts)
        if self._prev_counts is None or self._prev_t is None:
            self._prev_counts = counts
            self._prev_t = now
            return

        dt = now - self._prev_t
        if dt <= 0:
            return

        cps = {
            m: abs(counts[m] - self._prev_counts.get(m, counts[m])) / dt
            for m in counts
        }
        self._prev_counts = counts
        self._prev_t = now

        if sides is None:
            return

        self.moving_samples += 1
        left_cmd, right_cmd = sides
        left_motors = [m for m in rover_mod.LEFT_MOTORS if m in cps]
        right_motors = [m for m in rover_mod.RIGHT_MOTORS if m in cps]
        mean_L = statistics.mean(cps[m] for m in left_motors) if left_motors else 0.0
        mean_R = statistics.mean(cps[m] for m in right_motors) if right_motors else 0.0
        self.rates_L.append(mean_L)
        self.rates_R.append(mean_R)

        stalled = []
        if abs(left_cmd) >= CMD_EPS and mean_L < STALL_CPS:
            stalled.append(f"L(cmd={left_cmd:+.2f},cps={mean_L:.0f})")
        if abs(right_cmd) >= CMD_EPS and mean_R < STALL_CPS:
            stalled.append(f"R(cmd={right_cmd:+.2f},cps={mean_R:.0f})")

        if stalled:
            self.stall_samples += 1
            # Log at most ~4 Hz of stall lines to keep paste size sane.
            if self.elapsed() - self._last_stall_log_t >= 0.25:
                self._last_stall_log_t = self.elapsed()
                detail = " ".join(stalled)
                per = " ".join(f"M{m}={cps[m]:.0f}" for m in sorted(cps))
                line = (
                    f"t={self.elapsed():6.2f}s  STALL  {detail}  "
                    f"keys_cmd={self._last_cmd}  {per}"
                )
                self.stall_events.append(line)
                self.events.append(line)

    def write(self, path: Path = LOG_PATH) -> Path:
        moving = max(self.moving_samples, 1)
        stall_frac = self.stall_samples / moving
        mean_L = statistics.mean(self.rates_L) if self.rates_L else 0.0
        mean_R = statistics.mean(self.rates_R) if self.rates_R else 0.0
        lines = [
            "=" * 72,
            "PASTE THIS BLOCK BACK TO CURSOR",
            "=" * 72,
            f"script=wasd_arc.py  duration_s={self.elapsed():.1f}",
            f"drive_th={self.drive_th}  arc_inner={self.arc_inner}",
            f"samples={self.samples}  moving_samples={self.moving_samples}  "
            f"stall_samples={self.stall_samples}  stall_frac={stall_frac:.3f}",
            f"mean_cps while moving: L={mean_L:.0f}  R={mean_R:.0f}",
            f"stall_threshold_cps={STALL_CPS}",
            "",
            f"stall_events ({len(self.stall_events)}):",
        ]
        if self.stall_events:
            lines.extend(self.stall_events[:80])
            if len(self.stall_events) > 80:
                lines.append(f"  ... ({len(self.stall_events) - 80} more truncated)")
        else:
            lines.append("  (none)")
        lines.append("")
        lines.append(f"command timeline ({len(self.events)} lines, stalls included):")
        # Prefer command changes; include stalls already in events.
        cmd_lines = [e for e in self.events if " CMD " in e]
        lines.extend(cmd_lines[:60])
        if len(cmd_lines) > 60:
            lines.append(f"  ... ({len(cmd_lines) - 60} more CMD lines truncated)")
        lines.append("=" * 72)
        text = "\n".join(lines) + "\n"
        path.write_text(text)
        return path


def main():
    drive_th = 0.5
    arc_inner = 0.4
    timeout = 0.40
    step_mm = 50.0
    log = SessionLog()
    log.drive_th = drive_th
    log.arc_inner = arc_inner

    stop_on_enter._stop = threading.Event()

    print("WASD arc control (no spin-in-place)")
    print("  Hold W forward, S back.")
    print("  Hold W+A arc left, W+D arc right (S+A / S+D reverse).")
    print("  A or D alone = pivot on inside wheels.")
    print("  Tip: keep both chord keys repeating within timeout.")
    print("  -/= drive th   [/] arc inner   1/2 hold timeout")
    print("  e/c encoder drive nudge   3/4 nudge mm   Enter quit")
    print(f"  Session log → {LOG_PATH.name} (paste that file back)\n")
    status(drive_th, arc_inner, timeout, step_mm)
    print()

    bot = rover_mod.Rover()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    last_seen: dict[str, float] = {}
    pending = None
    last_sides = object()

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
                    log.note_cmd("nudge +drive")
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
                    log.note_cmd("nudge -drive")
                    result = run_nudge(
                        bot, lambda: bot.drive(-step_mm, throttle=drive_th)
                    )
                    if result == "\n":
                        break
                    if result:
                        pending = result
                elif ch == "-" or key == "_":
                    drive_th = clamp_throttle(drive_th - THROTTLE_STEP)
                    log.drive_th = drive_th
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch in ("=", "+"):
                    drive_th = clamp_throttle(drive_th + THROTTLE_STEP)
                    log.drive_th = drive_th
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "[":
                    arc_inner = clamp_arc(arc_inner - ARC_STEP)
                    log.arc_inner = arc_inner
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "]":
                    arc_inner = clamp_arc(arc_inner + ARC_STEP)
                    log.arc_inner = arc_inner
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

            sides = sides_from_keys(active, drive_th, arc_inner)
            if sides != last_sides:
                last_sides = sides
                log.note_cmd(cmd_label(active, sides))
            apply_sides(bot, sides)
            log.sample(bot, sides)
    finally:
        stop_on_enter._stop.set()
        bot.stop()
        try:
            bot.cleanup()
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        path = log.write()
        print(f"Quit. Log written to {path}")
        print(f"Paste with:  cat {path.name}")


if __name__ == "__main__":
    main()
