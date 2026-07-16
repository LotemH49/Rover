"""Standalone bench test for Motor HAT channel 1 + its encoder.

Completely independent of rover.py. Run on the Pi:

    python3 motor_test.py

Hardware under test (T-Cobbler layout from this project's wiring plan):
    - HAT terminal: motor1 (M1)
    - Encoder A -> GPIO5  (physical pin 29)
    - Encoder B -> GPIO6  (physical pin 31)
    - Encoder GND / 3.3V to Pi common ground and 3.3V

What it checks:
    1. Motor spins when throttle is applied (encoder counts change)
    2. Forward throttle produces positive count delta (A/B sense / MOTOR_SIGN)
    3. Reverse throttle produces negative count delta
"""

import sys
import time

import board  # pyright: ignore[reportMissingImports]
import RPi.GPIO as GPIO  # pyright: ignore[reportMissingImports, reportMissingModuleSource]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

# Motor 1 only — match rover ENC_PINS[1]
ENC_A = 5
ENC_B = 6
# Same convention as rover.MOTOR_SIGN[1]
MOTOR_SIGN = -1

THROTTLE = 0.4
SPIN_SECONDS = 1.5
MIN_COUNTS = 50  # enough to prove motion; ~1 revolution is ~900


class Motor1Test:
    def __init__(self):
        self.kit = MotorKit(i2c=board.I2C())
        self.motor = self.kit.motor1
        self.count = 0
        self._setup_encoder()
        self.motor.throttle = 0

    def _setup_encoder(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        def callback(channel):
            a = GPIO.input(ENC_A)
            b = GPIO.input(ENC_B)
            if channel == ENC_A:
                self.count += 1 if a != b else -1
            else:
                self.count += 1 if a == b else -1

        GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=callback)
        GPIO.add_event_detect(ENC_B, GPIO.BOTH, callback=callback)

    def _spin(self, logical_throttle, seconds):
        """Apply logical forward(+)/backward(-) throttle for ``seconds``."""
        self.count = 0
        self.motor.throttle = MOTOR_SIGN * logical_throttle
        time.sleep(seconds)
        self.motor.throttle = 0
        time.sleep(0.2)
        return self.count

    def cleanup(self):
        self.motor.throttle = 0
        GPIO.cleanup()


def _report(ok, message):
    tag = "PASS" if ok else "FAIL"
    print(f"  [{tag}] {message}")
    return ok


def run():
    print("Motor 1 wiring test")
    print(f"  HAT: motor1  |  encoder A=GPIO{ENC_A}  B=GPIO{ENC_B}")
    print(f"  throttle ±{THROTTLE} for {SPIN_SECONDS}s each direction")
    print()

    test = Motor1Test()
    passed = True

    try:
        print("1) Forward spin...")
        fwd = test._spin(+THROTTLE, SPIN_SECONDS)
        moved = abs(fwd) >= MIN_COUNTS
        passed &= _report(
            moved,
            f"encoder saw motion ({fwd} counts; need |delta| >= {MIN_COUNTS})",
        )
        if not moved:
            print(
                "       Check: M1 power wires on HAT, encoder VCC/GND, "
                f"A/B on GPIO{ENC_A}/{ENC_B}, battery power to HAT."
            )
        else:
            direction_ok = fwd > 0
            passed &= _report(
                direction_ok,
                f"forward counts positive (got {fwd})",
            )
            if not direction_ok:
                print(
                    "       Counts went the wrong way. Swap encoder A/B wires, "
                    "or swap the two motor power leads on M1."
                )

        print()
        print("2) Reverse spin...")
        rev = test._spin(-THROTTLE, SPIN_SECONDS)
        moved = abs(rev) >= MIN_COUNTS
        passed &= _report(
            moved,
            f"encoder saw motion ({rev} counts; need |delta| >= {MIN_COUNTS})",
        )
        if moved:
            direction_ok = rev < 0
            passed &= _report(
                direction_ok,
                f"reverse counts negative (got {rev})",
            )
            if not direction_ok:
                print(
                    "       Reverse sense does not match forward. "
                    "Double-check A/B wiring and motor leads."
                )

    finally:
        test.cleanup()

    print()
    if passed:
        print("All checks passed — motor 1 is wired correctly.")
        return 0

    print("One or more checks failed — fix wiring and re-run.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(run())
    except KeyboardInterrupt:
        print("\nAborted.")
        GPIO.cleanup()
        sys.exit(130)
