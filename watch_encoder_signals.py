"""Live encoder signal watcher (no motors required).

Polls cobbler encoder GPIOs and prints whenever a level changes.
Spin a wheel by hand (encoder powered) and watch A/B toggle.

Run on the Pi:

    python3 watch_encoder_signals.py

Enter to quit.
"""

import select
import sys
import termios
import time
import tty

import lgpio  # pyright: ignore[reportMissingImports]

PINS = {
    16: "GPIO16 M1-FR-A",
    20: "GPIO20 M1-FR-B",
    23: "GPIO23 M2-FL-A",
    24: "GPIO24 M2-FL-B",
    27: "GPIO27 M3-RL-A",
    17: "GPIO17 M3-RL-B",
    26: "GPIO26 M4-RR-A",
    19: "GPIO19 M4-RR-B",
}


def open_gpiochip():
    last_err = None
    for chip in (4, 0):
        try:
            return lgpio.gpiochip_open(chip), chip
        except Exception as exc:
            last_err = exc
    raise RuntimeError(f"Could not open gpiochip: {last_err}")


def main():
    handle, chip = open_gpiochip()
    print(f"Watching encoder GPIOs on /dev/gpiochip{chip}")
    print("Spin a wheel by hand (encoder light on). Enter to quit.\n")

    for pin in PINS:
        lgpio.gpio_claim_input(handle, pin, lgpio.SET_PULL_UP)

    last = {pin: lgpio.gpio_read(handle, pin) for pin in PINS}
    for pin, level in last.items():
        print(f"  {PINS[pin]} = {level}")
    print()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            if select.select([sys.stdin], [], [], 0.02)[0]:
                if sys.stdin.read(1) in ("\n", "\r"):
                    break
            for pin in PINS:
                level = lgpio.gpio_read(handle, pin)
                if level != last[pin]:
                    print(
                        f"  {PINS[pin]} {last[pin]} -> {level}",
                        flush=True,
                    )
                    last[pin] = level
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        lgpio.gpiochip_close(handle)
        print("Quit.")


if __name__ == "__main__":
    main()
