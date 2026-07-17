#!/usr/bin/env python3
"""WASD control with arc turns (no spin-in-place).

Hold keys (terminal autorepeat); chords:

  W       = forward
  S       = back
  W+A     = arc left  (left/inner slower)
  W+D     = arc right
  S+A/S+D = reverse arcs
  A / D   = pivot on inside wheels (outside drives, inside stopped)

Terminals usually only autorepeat the *last* key of a chord. This script
cross-refreshes latches so holding WA/WD stays an arc instead of flickering
to pivot/stop. Motor throttles also slew instead of hard-switching.

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
TIMEOUT_MIN, TIMEOUT_MAX = 0.20, 2.0
ARC_STEP = 0.1
ARC_MIN, ARC_MAX = 0.0, 0.9
DIST_STEP = 10.0
DIST_MIN, DIST_MAX = 10.0, 500.0

# Key latch: how long a key stays active without a fresh press/repeat.
DEFAULT_HOLD_S = 0.70
# After one chord partner is released, keep the other latched this long
# so the user can re-assert it (W often stops repeating once A is held).
RELEASE_GRACE_S = 0.55
# Max throttle change per second (each side), softens stick-slip spikes.
SLEW_PER_S = 2.5

SAMPLE_DT = 0.05
STALL_CPS = 25.0
# Ignore low throttle while slewing up/down from stop (avoids false stalls).
# Keep ≤ typical arc-inner cmd (drive_th * arc_inner), e.g. 0.5*0.4=0.20.
CMD_EPS = 0.15
# Only score stalls once applied throttle is near the target command.
SETTLED_EPS = 0.08
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


def sides_from_axes(drive, turn, drive_th, arc_inner):
    """drive: 'w'|'s'|None  turn: 'a'|'d'|None → (left, right) or None."""
    if drive == "w":
        base = drive_th
    elif drive == "s":
        base = -drive_th
    else:
        base = 0.0

    if base != 0.0:
        if turn == "a":
            return (base * arc_inner, base)
        if turn == "d":
            return (base, base * arc_inner)
        return (base, base)

    if turn == "a":
        return (0.0, drive_th)
    if turn == "d":
        return (drive_th, 0.0)
    return None


def cmd_label(drive, turn, sides) -> str:
    if sides is None:
        return "stop"
    keys = "".join(k for k in (drive, turn) if k)
    return f"keys={keys or '-'} L={sides[0]:+.2f} R={sides[1]:+.2f}"


def apply_sides(bot, sides):
    if sides is None:
        bot.stop()
    else:
        bot._drive_sides(sides[0], sides[1])


def slew_toward(current, target, dt, rate):
    """Move current (L,R) toward target by at most rate*dt per side."""
    if target is None:
        target = (0.0, 0.0)
    if current is None:
        current = (0.0, 0.0)
    max_step = rate * dt
    out = []
    for c, t in zip(current, target):
        delta = t - c
        if abs(delta) <= max_step:
            out.append(t)
        else:
            out.append(c + max_step if delta > 0 else c - max_step)
    if abs(out[0]) < 0.01 and abs(out[1]) < 0.01 and target == (0.0, 0.0):
        return None
    return (out[0], out[1])


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


class ChordLatch:
    """Drive/turn latches that survive single-key autorepeat."""

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self.drive: str | None = None
        self.turn: str | None = None
        self.drive_until = 0.0
        self.turn_until = 0.0
        self._was_chord = False

    def clear(self) -> None:
        self.drive = None
        self.turn = None
        self.drive_until = 0.0
        self.turn_until = 0.0
        self._was_chord = False

    def note_key(self, key: str, now: float) -> None:
        hold = self.hold_s
        if key in ("w", "s"):
            if self.drive and self.drive != key:
                # Opposite drive replaces.
                self.turn = None
                self.turn_until = 0.0
            self.drive = key
            self.drive_until = now + hold
            # While W/S repeats, keep an existing turn chord alive.
            if self.turn is not None:
                self.turn_until = now + hold
        elif key in ("a", "d"):
            if self.turn and self.turn != key:
                pass  # opposite turn replaces below
            self.turn = key
            self.turn_until = now + hold
            # While A/D repeats, keep an existing drive chord alive.
            # (Fixes: only the last chord key autorepeats.)
            if self.drive is not None:
                self.drive_until = now + hold

    def poll(self, now: float) -> tuple[str | None, str | None]:
        drive_alive = self.drive is not None and now <= self.drive_until
        turn_alive = self.turn is not None and now <= self.turn_until

        if drive_alive and turn_alive:
            self._was_chord = True
        elif self._was_chord:
            # One partner dropped: grace so the remaining axis can be re-held.
            if drive_alive and not turn_alive:
                self.drive_until = max(self.drive_until, now + RELEASE_GRACE_S)
                self._was_chord = False
            elif turn_alive and not drive_alive:
                self.turn_until = max(self.turn_until, now + RELEASE_GRACE_S)
                self._was_chord = False
            else:
                self._was_chord = False

        drive = self.drive if (self.drive and now <= self.drive_until) else None
        turn = self.turn if (self.turn and now <= self.turn_until) else None
        if drive is None:
            self.drive = None
        if turn is None:
            self.turn = None
        return drive, turn


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
        self.arc_inner = 0.2

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def note_cmd(self, label: str) -> None:
        if label == self._last_cmd:
            return
        self._last_cmd = label
        self.events.append(f"t={self.elapsed():6.2f}s  CMD  {label}")

    def sample(self, bot, sides, target_sides=None) -> None:
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

        # Skip stall scoring until slew has nearly reached the intended command.
        if target_sides is None:
            return
        if abs(sides[0] - target_sides[0]) > SETTLED_EPS:
            return
        if abs(sides[1] - target_sides[1]) > SETTLED_EPS:
            return

        stalled = []
        if abs(left_cmd) >= CMD_EPS and mean_L < STALL_CPS:
            stalled.append(f"L(cmd={left_cmd:+.2f},cps={mean_L:.0f})")
        if abs(right_cmd) >= CMD_EPS and mean_R < STALL_CPS:
            stalled.append(f"R(cmd={right_cmd:+.2f},cps={mean_R:.0f})")

        if stalled:
            self.stall_samples += 1
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
            f"stall_threshold_cps={STALL_CPS}  cmd_eps={CMD_EPS}  "
            f"settled_eps={SETTLED_EPS}",
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
        cmd_lines = [e for e in self.events if " CMD " in e]
        lines.append(f"command timeline ({len(cmd_lines)} CMD lines):")
        lines.extend(cmd_lines[:60])
        if len(cmd_lines) > 60:
            lines.append(f"  ... ({len(cmd_lines) - 60} more CMD lines truncated)")
        lines.append("=" * 72)
        path.write_text("\n".join(lines) + "\n")
        return path


def main():
    drive_th = 0.5
    arc_inner = 0.2
    timeout = DEFAULT_HOLD_S
    step_mm = 50.0
    log = SessionLog()
    log.drive_th = drive_th
    log.arc_inner = arc_inner
    latch = ChordLatch(timeout)

    stop_on_enter._stop = threading.Event()

    print("WASD arc control (no spin-in-place)")
    print("  Hold W forward, S back.")
    print("  Hold W+A arc left, W+D arc right (S+A / S+D reverse).")
    print("  A or D alone = pivot on inside wheels.")
    print("  Chords stay latched even if only one key autorepeats.")
    print("  -/= drive th   [/] arc inner   1/2 hold timeout")
    print("  e/c encoder drive nudge   3/4 nudge mm   Enter quit")
    print(f"  Session log → {LOG_PATH.name} (paste that file back)\n")
    status(drive_th, arc_inner, timeout, step_mm)
    print()

    bot = rover_mod.Rover()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    pending = None
    target_sides = None
    applied_sides = None
    last_logged = object()
    last_loop = time.monotonic()

    try:
        tty.setcbreak(fd)
        while not stop_on_enter.stopped():
            ch = None
            if pending is not None:
                ch = pending
                pending = None
            elif select.select([sys.stdin], [], [], 0.02)[0]:
                ch = sys.stdin.read(1)

            now = time.monotonic()
            dt = min(0.05, max(0.001, now - last_loop))
            last_loop = now

            if ch is not None:
                if ch in ("\n", "\r"):
                    break
                key = ch.lower()

                if key in "wasd":
                    latch.note_key(key, now)
                elif key == "e":
                    bot.stop()
                    latch.clear()
                    target_sides = None
                    applied_sides = None
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
                    latch.clear()
                    target_sides = None
                    applied_sides = None
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
                    latch.hold_s = timeout
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "2":
                    timeout = clamp_timeout(timeout + TIMEOUT_STEP)
                    latch.hold_s = timeout
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "3":
                    step_mm = clamp_dist(step_mm - DIST_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)
                elif ch == "4":
                    step_mm = clamp_dist(step_mm + DIST_STEP)
                    status(drive_th, arc_inner, timeout, step_mm)

            drive, turn = latch.poll(now)
            target_sides = sides_from_axes(drive, turn, drive_th, arc_inner)
            applied_sides = slew_toward(applied_sides, target_sides, dt, SLEW_PER_S)

            # Log stable *target* intent (not every slew step).
            label = cmd_label(drive, turn, target_sides)
            if label != last_logged:
                last_logged = label
                log.note_cmd(label)

            apply_sides(bot, applied_sides)
            log.sample(bot, applied_sides, target_sides)
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
