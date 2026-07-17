#!/usr/bin/env python3
"""Drive a small square using one-side pivot turns (no spin-in-place).

Each corner: only the outside wheels run at PIVOT_THROTTLE (~0.6);
the inside side is stopped.

Default: 20 cm sides, four left (CCW) pivots of 90°.

Run on the Pi:

    python3 grid_pivot.py

Press Enter at any time to stop.
"""

import time

import stop_on_enter
import rover as rover_mod

SIDE_MM = 200.0
CORNER_DEG = 90.0
DRIVE_THROTTLE = 0.5
PIVOT_THROTTLE = 0.6
SIDES = 4
PAUSE_S = 0.25


def main():
    stop_on_enter.install()
    bot = rover_mod.Rover()

    try:
        print(
            f"Pivot square: {SIDE_MM:.0f} mm sides, "
            f"{CORNER_DEG:.0f}° one-side pivots @ {PIVOT_THROTTLE}"
        )
        for side in range(1, SIDES + 1):
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            print(f"Side {side}/{SIDES}: drive {SIDE_MM:.0f} mm @ {DRIVE_THROTTLE}")
            bot.drive(SIDE_MM, throttle=DRIVE_THROTTLE)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            time.sleep(PAUSE_S)

            print(
                f"Corner {side}/{SIDES}: pivot left {CORNER_DEG:.0f}° "
                f"(right side only @ {PIVOT_THROTTLE})"
            )
            bot.pivot(CORNER_DEG, throttle=PIVOT_THROTTLE)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            time.sleep(PAUSE_S)

        print("Grid square done.")
    finally:
        bot.stop()
        try:
            bot.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
