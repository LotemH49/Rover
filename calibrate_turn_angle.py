#!/usr/bin/env python3
"""Calibrate what encoder counts mean for a visual 90° turn.

For each trial the rover spins in place until you press Enter when you judge
it has turned 90°. The script records how far the encoders moved, so we can
compare that to the geometric estimate used by Rover.turn().

Usage (Pi):
  cd ~/Rover && git pull && source .venv/bin/activate
  python calibrate_turn_angle.py

Optional:
  python calibrate_turn_angle.py --throttle 0.5
  python calibrate_turn_angle.py --trials 3
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import stop_on_enter
from rover import (
    COUNTS_PER_MM,
    DEFAULT_TURN_THROTTLE,
    LEFT_MOTORS,
    RIGHT_MOTORS,
    TRACK_WIDTH_MM,
    Rover,
)


def geometric_90_target_counts() -> float:
    """What Rover.turn(90) currently aims for (mean abs encoder counts)."""
    arc_mm = math.radians(90.0) * (TRACK_WIDTH_MM / 2.0)
    return arc_mm * COUNTS_PER_MM


def angle_from_mean_counts(mean_abs: float) -> float:
    """Invert the geometric formula: counts → degrees."""
    if COUNTS_PER_MM <= 0 or TRACK_WIDTH_MM <= 0:
        return float("nan")
    arc_mm = mean_abs / COUNTS_PER_MM
    return math.degrees(arc_mm / (TRACK_WIDTH_MM / 2.0))


def side_mean(counts: dict[int, int], motors: tuple[int, ...]) -> float:
    vals = [abs(counts[m]) for m in motors if m in counts]
    return statistics.mean(vals) if vals else 0.0


def run_trial(rover: Rover, *, direction: int, throttle: float, label: str) -> dict:
    """Spin until Enter; return measurement dict. direction: +1=CCW, -1=CW."""
    print()
    print(f"=== {label} ===")
    print("Aim the rover at a fixed reference (tape line, wall corner, etc.).")
    print("Press Enter to START spinning…")
    input()

    speed = abs(throttle)
    left = -speed * direction
    right = speed * direction

    rover._reset_counts()
    stop_on_enter.rearm()
    t0 = time.monotonic()
    rover._drive_sides(left, right)
    print("Spinning… press Enter when you judge it has turned 90°.")
    try:
        while not stop_on_enter.stopped():
            time.sleep(0.01)
    finally:
        rover.stop()
        elapsed = time.monotonic() - t0

    counts = dict(rover.counts)
    mean_all = statistics.mean(abs(c) for c in counts.values()) if counts else 0.0
    mean_l = side_mean(counts, LEFT_MOTORS)
    mean_r = side_mean(counts, RIGHT_MOTORS)
    soft_angle = angle_from_mean_counts(mean_all)

    print(f"  stopped after {elapsed:.2f}s")
    print(f"  counts: " + "  ".join(f"M{m}={counts[m]:+d}" for m in sorted(counts)))
    print(f"  mean |counts| all={mean_all:.1f}  L={mean_l:.1f}  R={mean_r:.1f}")
    print(f"  geometric formula says that is {soft_angle:.1f}°")
    print(f"  (you marked this as 90° visually)")

    return {
        "label": label,
        "direction": "CCW" if direction > 0 else "CW",
        "throttle": speed,
        "elapsed_s": round(elapsed, 3),
        "counts": counts,
        "mean_abs": round(mean_all, 1),
        "mean_L": round(mean_l, 1),
        "mean_R": round(mean_r, 1),
        "geometric_angle_deg": round(soft_angle, 1),
    }


def print_report(trials: list[dict], geometric_target: float) -> None:
    print()
    print("=" * 72)
    print("PASTE THIS BLOCK BACK TO CURSOR")
    print("=" * 72)
    print(f"TRACK_WIDTH_MM={TRACK_WIDTH_MM}  COUNTS_PER_MM={COUNTS_PER_MM:.6f}")
    print(f"geometric Rover.turn(90) target mean |counts| = {geometric_target:.1f}")
    print()

    for t in trials:
        c = t["counts"]
        print(
            f"{t['label']}: {t['direction']} throttle={t['throttle']} "
            f"t={t['elapsed_s']}s mean|c|={t['mean_abs']} "
            f"L={t['mean_L']} R={t['mean_R']} "
            f"geom_angle={t['geometric_angle_deg']}° "
            f"counts={{{', '.join(f'M{m}:{c[m]:+d}' for m in sorted(c))}}}"
        )

    by_dir: dict[str, list[float]] = {"CCW": [], "CW": []}
    for t in trials:
        by_dir[t["direction"]].append(t["mean_abs"])

    print()
    print("Suggested calibration (user-marked 90° → mean |counts|):")
    for d, vals in by_dir.items():
        if not vals:
            continue
        avg = statistics.mean(vals)
        scale = avg / geometric_target if geometric_target else float("nan")
        print(
            f"  {d}: n={len(vals)}  avg_mean|c|={avg:.1f}  "
            f"vs geometric {geometric_target:.1f}  "
            f"scale_factor={scale:.3f}  "
            f"(if soft says 90°, multiply turn targets by {scale:.3f})"
        )
    print("=" * 72)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--throttle",
        type=float,
        default=DEFAULT_TURN_THROTTLE,
        help=f"in-place spin throttle (default {DEFAULT_TURN_THROTTLE})",
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=2,
        help="how many Enter-marked 90° trials per direction (default 2)",
    )
    args = parser.parse_args()

    geometric_target = geometric_90_target_counts()

    print("Turn-angle calibration (Enter = visual 90°)")
    print(f"  throttle={args.throttle}  trials/direction={args.trials}")
    print(f"  geometric 90° target ≈ {geometric_target:.1f} mean |counts|")
    print("  Tip: mark a line on the floor / use a wall corner as reference.")
    print("  While spinning: Enter marks 90° and stops.")

    rover = Rover()
    trials: list[dict] = []

    try:
        for i in range(1, args.trials + 1):
            trials.append(
                run_trial(
                    rover,
                    direction=+1,
                    throttle=args.throttle,
                    label=f"CCW trial {i}/{args.trials}",
                )
            )
        for i in range(1, args.trials + 1):
            trials.append(
                run_trial(
                    rover,
                    direction=-1,
                    throttle=args.throttle,
                    label=f"CW trial {i}/{args.trials}",
                )
            )
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        rover.stop()
        rover.cleanup()

    if trials:
        print_report(trials, geometric_target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
