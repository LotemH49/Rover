"""Characterize in-place turns and detect skid via encoder rate spikes.

Spins open-loop CCW then CW (not closed-loop rover.turn), samples each
working encoder every ~20 ms, and flags skid when count rate jumps above
a robust baseline median.

M4 encoder is omitted (dead). Paste the full report back into chat.

Run on the Pi:

    python3 characterize_turn.py

Press Enter at any time to stop.
"""

import statistics
import time

import stop_on_enter
import rover as rover_mod

THROTTLE = 0.5
DURATION_S = 3.0
SAMPLE_DT = 0.02
WARMUP_S = 0.4
PAUSE_S = 1.0
SPIKE_FACTOR = 1.8
SPIKE_FLOOR_CPS = 50.0   # ignore tiny noise spikes
STALL_CPS = 20.0         # commanded but almost stopped

MOTOR_NAMES = {
    1: "M1 FR",
    2: "M2 FL",
    3: "M3 RL",
    4: "M4 RR",
}


def median(values):
    if not values:
        return 0.0
    return float(statistics.median(values))


def run_direction(bot, label, left, right):
    """Open-loop spin; return sample list and event lists."""
    print(f"\n=== {label}: left={left:+.2f} right={right:+.2f} for {DURATION_S}s ===")
    bot._reset_counts()
    prev = dict(bot.counts)
    t0 = time.monotonic()
    t_prev = t0
    samples = []  # dicts with t, rates_cps, rates_mms
    rates_by_motor = {m: [] for m in bot.counts}
    spikes = []
    stalls = []

    bot._drive_sides(left, right)
    try:
        while True:
            if stop_on_enter.stopped():
                break
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= DURATION_S:
                break

            time.sleep(SAMPLE_DT)
            now = time.monotonic()
            dt = now - t_prev
            if dt <= 0:
                continue
            t_prev = now
            elapsed = now - t0

            cur = dict(bot.counts)
            rates_cps = {}
            rates_mms = {}
            for m in cur:
                dc = abs(cur[m] - prev[m])
                cps = dc / dt
                rates_cps[m] = cps
                rates_mms[m] = cps / rover_mod.COUNTS_PER_MM
                if elapsed >= WARMUP_S:
                    rates_by_motor[m].append(cps)
            prev = cur
            samples.append({
                "t": elapsed,
                "cps": rates_cps,
                "mms": rates_mms,
            })

            if elapsed < WARMUP_S:
                continue

            for m, cps in rates_cps.items():
                base = median(rates_by_motor[m][:-1] or rates_by_motor[m])
                if base > 0 and cps >= SPIKE_FLOOR_CPS and cps > SPIKE_FACTOR * base:
                    spikes.append({
                        "t": elapsed,
                        "motor": m,
                        "cps": cps,
                        "mms": rates_mms[m],
                        "baseline": base,
                        "ratio": cps / base,
                    })
                if cps < STALL_CPS:
                    stalls.append({
                        "t": elapsed,
                        "motor": m,
                        "cps": cps,
                    })
    finally:
        bot.stop()

    return {
        "label": label,
        "left": left,
        "right": right,
        "samples": samples,
        "rates_by_motor": rates_by_motor,
        "spikes": spikes,
        "stalls": stalls,
    }


def summarize(result):
    lines = []
    lines.append(f"DIRECTION: {result['label']}")
    lines.append(f"  command: left={result['left']:+.2f}  right={result['right']:+.2f}")

    post = {}
    for m, rates in result["rates_by_motor"].items():
        # rates_by_motor only collected after warmup already
        if not rates:
            lines.append(f"  {MOTOR_NAMES.get(m, m)}: no samples")
            continue
        mean_cps = statistics.mean(rates)
        max_cps = max(rates)
        mean_mms = mean_cps / rover_mod.COUNTS_PER_MM
        max_mms = max_cps / rover_mod.COUNTS_PER_MM
        post[m] = mean_cps
        lines.append(
            f"  {MOTOR_NAMES.get(m, m)}: "
            f"mean={mean_cps:.0f} cps ({mean_mms:.0f} mm/s)  "
            f"max={max_cps:.0f} cps ({max_mms:.0f} mm/s)  "
            f"n={len(rates)}"
        )

    # Left = M2+M3, right = M1 (M4 encoder missing)
    left_ids = [m for m in (2, 3) if m in post]
    right_ids = [m for m in (1, 4) if m in post]
    if left_ids and right_ids:
        left_mean = statistics.mean(post[m] for m in left_ids)
        right_mean = statistics.mean(post[m] for m in right_ids)
        if right_mean > 1:
            ratio = left_mean / right_mean
            lines.append(
                f"  L/R mean-rate ratio: {ratio:.2f}  "
                f"(left={left_mean:.0f} cps, right={right_mean:.0f} cps)"
            )
        else:
            lines.append("  L/R mean-rate ratio: n/a (right ~0)")

    spikes = result["spikes"]
    if not spikes:
        lines.append("  skid spikes: none")
    else:
        lines.append(f"  skid spikes: {len(spikes)}")
        # Collapse per motor
        by_m = {}
        for s in spikes:
            by_m.setdefault(s["motor"], []).append(s)
        for m, evs in sorted(by_m.items()):
            worst = max(evs, key=lambda e: e["ratio"])
            lines.append(
                f"    {MOTOR_NAMES.get(m, m)}: {len(evs)} events, "
                f"worst t={worst['t']:.2f}s rate={worst['cps']:.0f} cps "
                f"({worst['ratio']:.1f}x baseline {worst['baseline']:.0f})"
            )

    stall_motors = sorted({s["motor"] for s in result["stalls"]})
    if stall_motors:
        names = ", ".join(MOTOR_NAMES.get(m, str(m)) for m in stall_motors)
        n = len(result["stalls"])
        lines.append(f"  stall samples: {n} across {names}")
    else:
        lines.append("  stall samples: none")

    # Verdict
    if spikes:
        offenders = sorted(
            {MOTOR_NAMES.get(s["motor"], s["motor"]) for s in spikes}
        )
        lines.append(f"  VERDICT: SKID suspected on {', '.join(offenders)}")
    else:
        lines.append("  VERDICT: no rate spikes above threshold")

    return "\n".join(lines)


def main():
    stop_on_enter.install()
    print("Turn skid characterization")
    print(f"  throttle={THROTTLE}  duration={DURATION_S}s  dt={SAMPLE_DT}s")
    print(f"  warmup={WARMUP_S}s  spike_factor={SPIKE_FACTOR}  floor={SPIKE_FLOOR_CPS} cps")
    print(f"  encoders: {sorted(rover_mod.ENC_PINS.keys())}  (M4 omitted if dead)")
    print("  Paste everything below this line back into chat.\n")
    print("=" * 60)

    bot = rover_mod.Rover()
    results = []
    try:
        # CCW: left backward, right forward (same as rover.turn positive)
        results.append(run_direction(bot, "CCW / left", -THROTTLE, +THROTTLE))
        if stop_on_enter.stopped():
            print("\nStopped early.")
        else:
            print(f"Pause {PAUSE_S}s...")
            if stop_on_enter.sleep(PAUSE_S):
                print("\nStopped early.")
            else:
                # CW: left forward, right backward
                results.append(
                    run_direction(bot, "CW / right", +THROTTLE, -THROTTLE)
                )

        print("\n" + "=" * 60)
        print("REPORT (copy from here)")
        print("=" * 60)
        print(f"throttle={THROTTLE} duration={DURATION_S}s sample_dt={SAMPLE_DT}s")
        print(f"spike_factor={SPIKE_FACTOR} warmup={WARMUP_S}s")
        print(f"working_encoders={sorted(rover_mod.ENC_PINS.keys())}")
        print()
        for r in results:
            print(summarize(r))
            print()
        print("=" * 60)
        print("END REPORT")
        print("=" * 60)
    finally:
        bot.stop()
        try:
            bot.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
