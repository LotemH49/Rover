"""Full turn skid characterization suite.

Sweeps several throttles, spins open-loop CCW and CW at each, samples
per-wheel encoder rate, flags spikes/stalls/asymmetry, and prints a
paste-ready report plus a recommended turn throttle.

M4 encoder omitted (dead). Enter stops early.

Run on the Pi:

    python3 characterize_turn.py
"""

import statistics
import time

import stop_on_enter
import rover as rover_mod

# Throttle ladder — find the gentlest turn that still moves cleanly.
THROTTLES = [0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00]
DURATION_S = 2.5
SAMPLE_DT = 0.02
WARMUP_S = 0.4
PAUSE_S = 0.8
SPIKE_FACTOR = 1.8
SPIKE_FLOOR_CPS = 50.0
STALL_CPS = 20.0

# Asymmetry: L/R ratio outside [LOW, HIGH] counts as bad.
LR_LOW = 0.70
LR_HIGH = 1.30

# "Clean enough" if spikes are rare relative to samples after warmup.
MAX_SPIKE_FRAC = 0.08

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


def run_direction(bot, label, left, right, throttle):
    """Open-loop spin; return metrics dict."""
    print(
        f"\n=== throttle={throttle:.2f} {label}: "
        f"left={left:+.2f} right={right:+.2f} for {DURATION_S}s ===",
        flush=True,
    )
    bot._reset_counts()
    prev = dict(bot.counts)
    t0 = time.monotonic()
    t_prev = t0
    rates_by_motor = {m: [] for m in bot.counts}
    spikes = []
    stalls = []
    n_post = 0

    bot._drive_sides(left, right)
    try:
        while True:
            if stop_on_enter.stopped():
                break
            now = time.monotonic()
            if now - t0 >= DURATION_S:
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
            for m in cur:
                cps = abs(cur[m] - prev[m]) / dt
                rates_cps[m] = cps
                if elapsed >= WARMUP_S:
                    rates_by_motor[m].append(cps)
            prev = cur

            if elapsed < WARMUP_S:
                continue

            n_post += 1
            for m, cps in rates_cps.items():
                hist = rates_by_motor[m]
                base = median(hist[:-1] or hist)
                if base > 0 and cps >= SPIKE_FLOOR_CPS and cps > SPIKE_FACTOR * base:
                    spikes.append({
                        "t": elapsed,
                        "motor": m,
                        "cps": cps,
                        "baseline": base,
                        "ratio": cps / base,
                    })
                if cps < STALL_CPS:
                    stalls.append({"t": elapsed, "motor": m, "cps": cps})
    finally:
        bot.stop()

    means = {}
    maxes = {}
    for m, rates in rates_by_motor.items():
        if rates:
            means[m] = statistics.mean(rates)
            maxes[m] = max(rates)
        else:
            means[m] = 0.0
            maxes[m] = 0.0

    left_ids = [m for m in (2, 3) if m in means]
    right_ids = [m for m in (1, 4) if m in means]
    left_mean = statistics.mean(means[m] for m in left_ids) if left_ids else 0.0
    right_mean = statistics.mean(means[m] for m in right_ids) if right_ids else 0.0
    if right_mean > 1:
        lr_ratio = left_mean / right_mean
    else:
        lr_ratio = float("inf") if left_mean > 1 else 1.0

    spike_frac = (len(spikes) / n_post) if n_post else 0.0
    asymmetric = not (LR_LOW <= lr_ratio <= LR_HIGH)
    spiked = spike_frac > MAX_SPIKE_FRAC
    stalled = len(stalls) > n_post * 0.05 if n_post else False
    clean = (not asymmetric) and (not spiked) and (not stalled) and left_mean > 50 and right_mean > 50

    return {
        "throttle": throttle,
        "label": label,
        "left_cmd": left,
        "right_cmd": right,
        "means": means,
        "maxes": maxes,
        "left_mean": left_mean,
        "right_mean": right_mean,
        "lr_ratio": lr_ratio,
        "spikes": spikes,
        "stalls": stalls,
        "n_post": n_post,
        "spike_frac": spike_frac,
        "asymmetric": asymmetric,
        "spiked": spiked,
        "stalled": stalled,
        "clean": clean,
    }


def summarize(result):
    lines = []
    th = result["throttle"]
    lines.append(f"THROTTLE {th:.2f}  DIRECTION: {result['label']}")
    lines.append(
        f"  command: left={result['left_cmd']:+.2f}  right={result['right_cmd']:+.2f}"
    )
    for m in sorted(result["means"]):
        mean_cps = result["means"][m]
        max_cps = result["maxes"][m]
        lines.append(
            f"  {MOTOR_NAMES.get(m, m)}: "
            f"mean={mean_cps:.0f} cps ({mean_cps / rover_mod.COUNTS_PER_MM:.0f} mm/s)  "
            f"max={max_cps:.0f} cps ({max_cps / rover_mod.COUNTS_PER_MM:.0f} mm/s)"
        )
    lr = result["lr_ratio"]
    lr_s = f"{lr:.2f}" if lr != float("inf") else "inf"
    lines.append(
        f"  L/R mean-rate ratio: {lr_s}  "
        f"(left={result['left_mean']:.0f} cps, right={result['right_mean']:.0f} cps)  "
        f"{'BAD' if result['asymmetric'] else 'ok'}"
    )

    spikes = result["spikes"]
    if not spikes:
        lines.append("  skid spikes: none")
    else:
        lines.append(
            f"  skid spikes: {len(spikes)}  "
            f"(frac={result['spike_frac']:.2f}, "
            f"{'BAD' if result['spiked'] else 'ok'})"
        )
        by_m = {}
        for s in spikes:
            by_m.setdefault(s["motor"], []).append(s)
        for m, evs in sorted(by_m.items()):
            worst = max(evs, key=lambda e: e["ratio"])
            lines.append(
                f"    {MOTOR_NAMES.get(m, m)}: {len(evs)} events, "
                f"worst t={worst['t']:.2f}s "
                f"{worst['cps']:.0f} cps ({worst['ratio']:.1f}x baseline)"
            )

    if result["stalls"]:
        names = sorted({MOTOR_NAMES.get(s["motor"], s["motor"]) for s in result["stalls"]})
        lines.append(
            f"  stall samples: {len(result['stalls'])} across {', '.join(names)}  "
            f"{'BAD' if result['stalled'] else 'ok'}"
        )
    else:
        lines.append("  stall samples: none")

    if result["clean"]:
        lines.append("  VERDICT: CLEAN")
    else:
        bits = []
        if result["asymmetric"]:
            bits.append("asymmetric")
        if result["spiked"]:
            bits.append("spikes")
        if result["stalled"]:
            bits.append("stalls")
        if result["left_mean"] <= 50 or result["right_mean"] <= 50:
            bits.append("too_slow")
        lines.append(f"  VERDICT: PROBLEMS ({', '.join(bits) or 'unknown'})")

    return "\n".join(lines)


def recommend(results):
    lines = ["RECOMMENDATION"]
    # Prefer lowest throttle where BOTH directions are clean.
    by_th = {}
    for r in results:
        by_th.setdefault(r["throttle"], []).append(r)

    both_clean = []
    one_clean = []
    for th in THROTTLES:
        rs = by_th.get(th, [])
        if not rs:
            continue
        if all(r["clean"] for r in rs) and len(rs) >= 2:
            both_clean.append(th)
        elif any(r["clean"] for r in rs):
            one_clean.append(th)

    if both_clean:
        best = min(both_clean)
        lines.append(
            f"  Use turn throttle ≈ {best:.2f} "
            f"(both CCW and CW clean at this level)."
        )
        lines.append(
            f"  Suggested rover/square TURN_THROTTLE = {best:.2f}"
        )
    elif one_clean:
        best = min(one_clean)
        lines.append(
            f"  Partial: throttle {best:.2f} clean in at least one direction only."
        )
        lines.append(
            f"  Try TURN_THROTTLE = {best:.2f} and re-test square on carpet/floor."
        )
    else:
        lines.append(
            "  No clean throttle in this sweep. "
            "Try a different surface, check FL bind, or arc turns instead of spin-in-place."
        )

    # Worst offenders overall
    spike_counts = {}
    for r in results:
        for s in r["spikes"]:
            spike_counts[s["motor"]] = spike_counts.get(s["motor"], 0) + 1
    if spike_counts:
        worst = sorted(spike_counts.items(), key=lambda kv: -kv[1])
        parts = [
            f"{MOTOR_NAMES.get(m, m)}={n}" for m, n in worst
        ]
        lines.append(f"  Most spike events overall: {', '.join(parts)}")

    asym = [
        r for r in results
        if r["asymmetric"] and r["lr_ratio"] != float("inf")
    ]
    if asym:
        worst_a = min(asym, key=lambda r: abs(1.0 - r["lr_ratio"]))
        # actually want farthest from 1
        worst_a = max(asym, key=lambda r: abs(1.0 - r["lr_ratio"]))
        lines.append(
            f"  Worst L/R asymmetry: throttle={worst_a['throttle']:.2f} "
            f"{worst_a['label']} ratio={worst_a['lr_ratio']:.2f}"
        )

    return "\n".join(lines)


def main():
    stop_on_enter.install()
    print("Turn skid characterization SUITE")
    print(f"  throttles={THROTTLES}")
    print(f"  duration={DURATION_S}s  dt={SAMPLE_DT}s  warmup={WARMUP_S}s")
    print(f"  spike_factor={SPIKE_FACTOR}  L/R ok=[{LR_LOW},{LR_HIGH}]")
    print(f"  encoders={sorted(rover_mod.ENC_PINS.keys())}")
    print("  Paste the REPORT section back into chat.\n")
    print("=" * 60)

    bot = rover_mod.Rover()
    results = []
    try:
        for th in THROTTLES:
            if stop_on_enter.stopped():
                break
            # CCW
            results.append(
                run_direction(bot, "CCW / left", -th, +th, th)
            )
            if stop_on_enter.stopped():
                break
            print(f"Pause {PAUSE_S}s...", flush=True)
            if stop_on_enter.sleep(PAUSE_S):
                break
            # CW
            results.append(
                run_direction(bot, "CW / right", +th, -th, th)
            )
            if stop_on_enter.stopped():
                break
            if th != THROTTLES[-1]:
                print(f"Pause {PAUSE_S}s before next throttle...", flush=True)
                if stop_on_enter.sleep(PAUSE_S):
                    break

        print("\n" + "=" * 60)
        print("REPORT (copy from here)")
        print("=" * 60)
        print(f"throttles={THROTTLES}")
        print(f"duration={DURATION_S}s sample_dt={SAMPLE_DT}s warmup={WARMUP_S}s")
        print(f"spike_factor={SPIKE_FACTOR} lr_ok=[{LR_LOW},{LR_HIGH}]")
        print(f"max_spike_frac={MAX_SPIKE_FRAC}")
        print(f"working_encoders={sorted(rover_mod.ENC_PINS.keys())}")
        print()
        for r in results:
            print(summarize(r))
            print()
        print(recommend(results))
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
