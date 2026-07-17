"""Map HAT motors to physical wheels + encoder GPIO pins.

Spins M1, then M2, then M3, then M4. While a motor runs, watches every
encoder GPIO for pulses. Press Enter to stop that motor, print which pins
saw activity, then move to the next motor.

Uses lgpio (Pi 5 compatible) — classic RPi.GPIO does not work on Pi 5.

After each step, note which physical wheel moved (FL / FR / RL / RR).
Paste the full terminal output back into chat so we can set MOTOR_SIGN /
ENC_PINS / LEFT_MOTORS correctly.

Run on the Pi:

    python3 map_motors_encoders.py

Press Enter after each motor. After M4, Enter quits.
"""

import select
import sys
import termios
import threading
import time
import tty

import board  # pyright: ignore[reportMissingImports]
import lgpio  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

# All candidate encoder GPIOs from the T-Cobbler plan (BCM).
CANDIDATE_PINS = {
    5: "GPIO5  (phys 29)",
    6: "GPIO6  (phys 31)",
    13: "GPIO13 (phys 33)",
    19: "GPIO19 (phys 35)",
    26: "GPIO26 (phys 37)",
    20: "GPIO20 (phys 38)",
    16: "GPIO16 (phys 36)",
    21: "GPIO21 (phys 40)",
}

THROTTLE = 0.4


def open_gpiochip():
    """Pi 5 uses gpiochip4; older Pis use gpiochip0."""
    last_err = None
    for chip in (4, 0):
        try:
            return lgpio.gpiochip_open(chip), chip
        except Exception as exc:
            last_err = exc
    raise RuntimeError(f"Could not open gpiochip: {last_err}")


class EncoderWatch:
    def __init__(self):
        self.counts = {pin: 0 for pin in CANDIDATE_PINS}
        self._lock = threading.Lock()
        self._callbacks = []
        self.handle, chip = open_gpiochip()
        print(f"  Using /dev/gpiochip{chip} (lgpio)\n")

        for pin in CANDIDATE_PINS:
            lgpio.gpio_claim_input(self.handle, pin, lgpio.SET_PULL_UP)
            cb = lgpio.callback(
                self.handle, pin, lgpio.BOTH_EDGES, self._make_cb(pin)
            )
            self._callbacks.append(cb)

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
    """Wait until Enter is pressed (cbreak mode)."""
    while True:
        if select.select([sys.stdin], [], [], 0.1)[0]:
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                return


def print_activity(counts):
    active = [(pin, n) for pin, n in counts.items() if n > 0]
    active.sort(key=lambda x: -x[1])
    if not active:
        print("  GPIO activity: NONE (no encoder pulses seen)")
        print("  -> Check encoder V (3.3V), GND, A/B wiring for this motor.")
        return

    print("  GPIO activity (pulses while this motor ran):")
    for pin, n in active:
        print(f"    {CANDIDATE_PINS[pin]}: {n} edges")
    top = [CANDIDATE_PINS[p] for p, _ in active[:2]]
    print(f"  Likely A/B pair: {', '.join(top)}")


def main():
    print("Motor + encoder mapper")
    print("  Each step: one HAT motor spins until you press Enter.")
    print("  Tell me which wheel moved (FL/FR/RL/RR) after each step.")
    print("  Paste this whole log back into chat when done.\n")

    kit = MotorKit(i2c=board.I2C())
    motors = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }
    watch = EncoderWatch()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        for num, motor in motors.items():
            watch.reset()
            print("=" * 60)
            print(f"STEP: HAT motor{num} (M{num}) — spinning now")
            print("  Which physical wheel is moving?")
            print("  (front-left / front-right / rear-left / rear-right)")
            print("  Press Enter when ready for the next motor...")
            print("=" * 60)

            motor.throttle = THROTTLE
            wait_enter()
            motor.throttle = 0
            time.sleep(0.15)

            counts = watch.snapshot()
            print(f"\nRESULT for HAT M{num}:")
            print_activity(counts)
            print(
                f"  >>> Write here: M{num} physical location = ________\n"
            )

        print("=" * 60)
        print("Done. Copy/paste everything above into chat.")
        print("Also fill in the four 'physical location' blanks.")
        print("=" * 60)
    finally:
        for motor in motors.values():
            motor.throttle = 0
        watch.cleanup()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
