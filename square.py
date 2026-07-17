"""Drive a 40 cm square using encoder closed-loop drive/turn.

Each side: drive 400 mm, then spin left 90 degrees. Four times.

Run on the Pi:

    python3 square.py

Press Enter at any time to stop.
"""

import stop_on_enter
import rover as rover_mod

SIDE_MM = 400.0          # 40 cm
CORNER_DEG = 90.0
DRIVE_THROTTLE = 0.5
TURN_THROTTLE = 0.5
SIDES = 4


def main():
    stop_on_enter.install()
    bot = rover_mod.Rover()

    try:
        print(f"Encoder square: {SIDE_MM:.0f} mm sides, {CORNER_DEG:.0f} deg corners")
        for side in range(1, SIDES + 1):
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            print(f"Side {side}/{SIDES}: drive {SIDE_MM:.0f} mm @ {DRIVE_THROTTLE}")
            bot.drive(SIDE_MM, throttle=DRIVE_THROTTLE)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            print(f"Side {side}/{SIDES}: turn left {CORNER_DEG:.0f} deg @ {TURN_THROTTLE}")
            bot.turn(CORNER_DEG, throttle=TURN_THROTTLE)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

        print("Square done.")
    finally:
        bot.stop()
        try:
            bot.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
