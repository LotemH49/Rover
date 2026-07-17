"""Hand-spin encoder pin finder.

You spin one wheel by hand; the script watches nearly all header GPIOs.
Press Enter to freeze counts and print which pins saw pulses, then do the
next wheel.

Skips I2C pins 2/3 (Motor HAT). Uses lgpio alerts (Pi 5).

Run on the Pi:

    python3 find_encoder_pins.py
"""

import select
import sys
import termios
import threading
import time
import tty

import lgpio  # pyright: ignore[reportMissingImports]

# BCM GPIOs on the 40-pin header (skip 2/3 = I2C for Motor HAT).
WATCH_PINS = [
    4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27,
]

# Physical pin lookup for printing (BCM -> header pin number).
BCM_TO_PHYS = {
    4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19, 11: 23,
    12: 32, 13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12, 19: 35,
    20: 38, 21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13,
}

STEPS = [
    "front-right (FR)",
    "front-left (FL)",
    "rear-left (RL / BL)",
    "rear-right (RR / BR)",
]


def open_gpiochip():
    last_err = None
    for chip in (4, 0):
        try:
            return lgpio.gpiochip_open(chip), chip
        except Exception as exc:
            last_err = exc
    raise RuntimeError(f"Could not open gpiochip: {last_err}")


class PinWatch:
    def __init__(self):
        self.counts = {pin: 0 for pin in WATCH_PINS}
        self._lock = threading.Lock()
        self._callbacks = []
        self.handle, chip = open_gpiochip()
        print(f"  Using /dev/gpiochip{chip}")
        print(f"  Watching BCM GPIOs: {WATCH_PINS}\n")

        for pin in WATCH_PINS:
            try:
                lgpio.gpio_claim_alert(
                    self.handle, pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP
                )
                cb = lgpio.callback(
                    self.handle, pin, lgpio.BOTH_EDGES, self._make_cb(pin)
                )
                self._callbacks.append(cb)
            except Exception as exc:
                print(f"  (skip GPIO{pin}: {exc})")

    def _make_cb(self, pin):
        def cb(_chip, _gpio, _level, _timestamp):
            with self._lock:
                self.counts[pin] += 1
        return cb

    def reset(self):
        with self._lock:
            for pin in self.counts:
                self.counts[pin] = 0

    def snapshot(self):
        with self._lock:
            return dict(self.counts)

    def cleanup(self):
        for cb in self._callbacks:
            try:
                cb.cancel()
            except Exception:
                pass
        try:
            lgpio.gpiochip_close(self.handle)
        except Exception:
            pass


def wait_enter():
    while True:
        if select.select([sys.stdin], [], [], 0.1)[0]:
            if sys.stdin.read(1) in ("\n", "\r"):
                return


def print_activity(counts):
    active = [(pin, n) for pin, n in counts.items() if n > 0]
    active.sort(key=lambda x: -x[1])
    if not active:
        print("  No GPIO edges seen.")
        print("  Check encoder light (3V3 + GND) and spin that wheel.")
        return

    print("  GPIO activity:")
    for pin, n in active:
        phys = BCM_TO_PHYS.get(pin, "?")
        print(f"    GPIO{pin:2d}  (phys {phys:2})  : {n} edges")
    top = [f"GPIO{p}" for p, _ in active[:2]]
    print(f"  Likely A/B: {', '.join(top)}")


def main():
    print("Hand-spin encoder pin finder")
    print("  For each wheel: spin by hand, then press Enter.")
    print("  Paste the full log back into chat when done.\n")

    watch = PinWatch()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        for i, name in enumerate(STEPS, start=1):
            watch.reset()
            print("=" * 60)
            print(f"STEP {i}/{len(STEPS)}: spin {name} by hand")
            print("  (encoder light should be on)")
            print("  Press Enter when done spinning...")
            print("=" * 60)
            wait_enter()
            time.sleep(0.05)
            print(f"\nRESULT for {name}:")
            print_activity(watch.snapshot())
            print()

        print("=" * 60)
        print("Done. Copy/paste everything above into chat.")
        print("=" * 60)
    finally:
        watch.cleanup()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
