"""Rover drive & turn primitives for a 4-motor Raspberry Pi rover.

This module exposes two high-level, encoder-closed-loop motions that the
radiation-search algorithms will call:

    rover.drive(distance_mm)   # straight line travel by distance
    rover.turn(angle_deg)      # spin in place by a heading change

Hardware:
    - Adafruit DC Motor HAT (I2C), motors addressed kit.motor1..4
    - 4x DFRobot FIT0522 gearmotors (12 CPR encoder, 75:1 gearbox)
    - Quadrature encoders via lgpio (Pi 5) or RPi.GPIO (sim / older Pi)

"""

import math
import time

import board  # pyright: ignore[reportMissingImports]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]

import stop_on_enter

try:
    import lgpio  # pyright: ignore[reportMissingImports]
    _HAS_LGPIO = True
except ImportError:  # simulator / older images
    import RPi.GPIO as GPIO  # pyright: ignore[reportMissingImports, reportMissingModuleSource]
    _HAS_LGPIO = False


# --------------------------------------------------------------------------
# Physical constants
# --------------------------------------------------------------------------
COUNTS_PER_REV = 900                      # 12 CPR encoder x 75:1 gearbox
WHEEL_DIAMETER_MM = 65.0
WHEEL_CIRC_MM = math.pi * WHEEL_DIAMETER_MM        # ~204.20 mm
TRACK_WIDTH_MM = 331.8                     # center-to-center, left<->right wheels
COUNTS_PER_MM = COUNTS_PER_REV / WHEEL_CIRC_MM     # ~4.41 counts per mm

# Logical "forward" sign for each motor. The right side is mounted mirrored,
# so driving forward means +throttle on the left and -throttle on the right.
# If any wheel spins the wrong way on the bench, flip its sign here (or swap
# its two wires in the HAT terminal block -- either fixes it).
# Verified HAT channel -> physical wheel (map_motors_encoders.py):
#   1 = front-right   2 = front-left   3 = rear-left   4 = rear-right
# Right-side motors are mirrored, so their signs are inverted.
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}

# Encoder GPIO pins (BCM numbering): motor -> (channel A, channel B).
# Wired on T-Cobbler: green = A, yellow = B (verified by hand).
#
#   Motor   Role          Chan A   Chan B   Cobbler labels    Phys pins
#   -----   ----          ------   ------   ---------------   ---------
#     1     front-right    16       20      GPIO16 / GPIO20   36 / 38
#     2     front-left     23       24      GPIO23 / GPIO24   16 / 18
#     3     rear-left      27       17      GPIO27 / GPIO17   13 / 11
#     4     rear-right     26       19      GPIO26 / GPIO19   37 / 35
#
# Share GND (phys 30/34/39) and encoder V from 3.3V (phys 1 or 17).
# M4 rear-right encoder is dead (likely fried); omit it so drive/turn
# average only the three working encoders. Motor 4 still drives.
ENC_PINS = {
    1: (16, 20),  # front-right
    2: (23, 24),  # front-left
    3: (27, 17),  # rear-left
    # 4: (26, 19),  # rear-right — disabled until replaced
}

# Left = front-left + rear-left; right = front-right + rear-right.
LEFT_MOTORS = (2, 3)
RIGHT_MOTORS = (1, 4)

# Default throttles -- modest so the rover is controllable; callers can override.
DEFAULT_DRIVE_THROTTLE = 0.5
DEFAULT_TURN_THROTTLE = 0.5

# Default lane spacing for basic_search (matches sim grid lines).
SEARCH_LANE_MM = 100


class Rover:
    """Four-motor rover with encoder-closed-loop drive and turn commands."""

    def __init__(self):
        self.kit = MotorKit(i2c=board.I2C())
        self._motors = {
            1: self.kit.motor1,
            2: self.kit.motor2,
            3: self.kit.motor3,
            4: self.kit.motor4,
        }

        # Encoder counts for motors listed in ENC_PINS only.
        self.counts = {name: 0 for name in ENC_PINS}
        self._gpio_handle = None
        self._gpio_callbacks = []

        self._setup_encoders()
        self.stop()

    # ------------------------------------------------------------------
    # Encoder layer
    # ------------------------------------------------------------------
    def _setup_encoders(self):
        if _HAS_LGPIO:
            self._setup_encoders_lgpio()
        else:
            self._setup_encoders_rpigpio()

    def _setup_encoders_lgpio(self):
        last_err = None
        for chip in (4, 0):
            try:
                self._gpio_handle = lgpio.gpiochip_open(chip)
                break
            except Exception as exc:
                last_err = exc
        if self._gpio_handle is None:
            raise RuntimeError(f"Could not open gpiochip: {last_err}")

        for name, (pin_a, pin_b) in ENC_PINS.items():
            # Pi 5: callbacks require alert claims (input-only claims stay silent).
            lgpio.gpio_claim_alert(
                self._gpio_handle, pin_a, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP
            )
            lgpio.gpio_claim_alert(
                self._gpio_handle, pin_b, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP
            )
            cb = self._make_callback_lgpio(name, pin_a, pin_b)
            self._gpio_callbacks.append(
                lgpio.callback(self._gpio_handle, pin_a, lgpio.BOTH_EDGES, cb)
            )
            self._gpio_callbacks.append(
                lgpio.callback(self._gpio_handle, pin_b, lgpio.BOTH_EDGES, cb)
            )

    def _setup_encoders_rpigpio(self):
        GPIO.setmode(GPIO.BCM)
        for name, (pin_a, pin_b) in ENC_PINS.items():
            GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            cb = self._make_callback_rpigpio(name, pin_a, pin_b)
            GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=cb)
            GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=cb)

    def _make_callback_lgpio(self, name, pin_a, pin_b):
        def callback(_chip, gpio, _level, _timestamp):
            a = lgpio.gpio_read(self._gpio_handle, pin_a)
            b = lgpio.gpio_read(self._gpio_handle, pin_b)
            if gpio == pin_a:
                self.counts[name] += 1 if a != b else -1
            else:
                self.counts[name] += 1 if a == b else -1
        return callback

    def _make_callback_rpigpio(self, name, pin_a, pin_b):
        def callback(channel):
            a = GPIO.input(pin_a)
            b = GPIO.input(pin_b)
            if channel == pin_a:
                self.counts[name] += 1 if a != b else -1
            else:
                self.counts[name] += 1 if a == b else -1
        return callback

    def _reset_counts(self):
        for name in self.counts:
            self.counts[name] = 0

    def _avg_abs_counts(self):
        """Mean absolute count across working encoders only (see ENC_PINS)."""
        return sum(abs(c) for c in self.counts.values()) / len(self.counts)

    def get_speed_mm_per_s(self, motor=1, interval=0.1):
        """Convenience: measured wheel speed in mm/s over a short interval."""
        start = self.counts[motor]
        time.sleep(interval)
        delta = self.counts[motor] - start
        return (delta / COUNTS_PER_REV) * WHEEL_CIRC_MM / interval

    # ------------------------------------------------------------------
    # Motor layer
    # ------------------------------------------------------------------
    def _set_logical(self, m1, m2, m3, m4):
        """Set per-HAT-channel throttles in plain forward(+)/backward(-) terms.

        Channels: 1=FR, 2=FL, 3=RL, 4=RR. MOTOR_SIGN applies mirroring.
        """
        logical = {1: m1, 2: m2, 3: m3, 4: m4}
        for name, value in logical.items():
            self._motors[name].throttle = MOTOR_SIGN[name] * value

    def _drive_sides(self, left, right):
        """Drive left (M2,M3) and right (M1,M4) at logical throttles."""
        self._set_logical(right, left, left, right)

    def stop(self):
        """Brake all motors (throttle 0)."""
        for motor in self._motors.values():
            motor.throttle = 0

    def coast(self):
        """Let all motors free-spin (throttle None)."""
        for motor in self._motors.values():
            motor.throttle = None

    # ------------------------------------------------------------------
    # High-level primitives
    # ------------------------------------------------------------------
    def drive(self, distance_mm, throttle=DEFAULT_DRIVE_THROTTLE):
        """Drive straight by distance_mm (negative = reverse), then stop.

        Uses the encoders for distance precision regardless of battery level.
        """
        if distance_mm == 0:
            return

        target = abs(distance_mm) * COUNTS_PER_MM
        speed = abs(throttle) * (1 if distance_mm > 0 else -1)

        self._reset_counts()
        self._drive_sides(speed, speed)
        try:
            while self._avg_abs_counts() < target:
                if stop_on_enter.stopped():
                    break
                time.sleep(0.005)
        finally:
            self.stop()

    def turn(self, angle_deg, throttle=DEFAULT_TURN_THROTTLE):
        """Spin in place by angle_deg, then stop.

        Convention: positive = counterclockwise (left turn),
                    negative = clockwise (right turn).

        Each wheel sweeps an arc of radius TRACK_WIDTH_MM / 2 about the rover
        center, so target distance per wheel = radians * (track / 2).
        """
        if angle_deg == 0:
            return

        arc_mm = math.radians(abs(angle_deg)) * (TRACK_WIDTH_MM / 2.0)
        target = arc_mm * COUNTS_PER_MM
        speed = abs(throttle)

        # CCW (positive): left side backward, right side forward.
        direction = 1 if angle_deg > 0 else -1
        left = -speed * direction
        right = speed * direction

        self._reset_counts()
        self._drive_sides(left, right)
        try:
            while self._avg_abs_counts() < target:
                if stop_on_enter.stopped():
                    break
                time.sleep(0.005)
        finally:
            self.stop()

    def pivot(self, angle_deg, throttle=0.6):
        """Pivot about one side: only the outside wheels drive.

        Convention: positive = counterclockwise (left turn) → right side drives,
                    left side stopped.
                    negative = clockwise (right turn) → left side drives.

        Moving wheels sweep roughly radius TRACK_WIDTH_MM about the inside.
        """
        if angle_deg == 0:
            return

        # Full track as pivot radius (inside wheels ≈ fixed).
        arc_mm = math.radians(abs(angle_deg)) * TRACK_WIDTH_MM
        target = arc_mm * COUNTS_PER_MM
        speed = abs(throttle)

        if angle_deg > 0:
            left, right = 0.0, speed
            moving = RIGHT_MOTORS
        else:
            left, right = speed, 0.0
            moving = LEFT_MOTORS

        self._reset_counts()
        self._drive_sides(left, right)
        try:
            while True:
                if stop_on_enter.stopped():
                    break
                vals = [abs(self.counts[m]) for m in moving if m in self.counts]
                if not vals:
                    # No encoder on moving side — time fallback (~rough).
                    break
                if sum(vals) / len(vals) >= target:
                    break
                time.sleep(0.005)
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def cleanup(self):
        """Stop motors and release GPIO. Call once at program exit."""
        self.stop()
        if _HAS_LGPIO:
            for cb in self._gpio_callbacks:
                try:
                    cb.cancel()
                except Exception:
                    pass
            if self._gpio_handle is not None:
                try:
                    lgpio.gpiochip_close(self._gpio_handle)
                except Exception:
                    pass
                self._gpio_handle = None
        else:
            GPIO.cleanup()


def _demo():
    """Bench demo: drive forward 500 mm, then turn left 90 degrees."""
    stop_on_enter.install()
    rover = Rover()
    try:
        print("Driving forward 500 mm...")
        rover.drive(500)
        if stop_on_enter.stopped():
            print("Stopped.")
            return
        if stop_on_enter.sleep(0.5):
            print("Stopped.")
            return

        print("Turning left 90 degrees...")
        rover.turn(90)
        if stop_on_enter.stopped():
            print("Stopped.")
            return
        if stop_on_enter.sleep(0.5):
            print("Stopped.")
            return

        print("Done.")
    finally:
        rover.cleanup()


def basic_search(x_max, y_max, angle_initial, rover=None, lane_mm=SEARCH_LANE_MM):
    """Lawnmower search over a rectangle up to (x_max, y_max).

    Sweeps horizontal rows of width x_max, stepping lane_mm between rows.
    Positive angle_initial rotates the rover before the first leg.
    """
    own_rover = rover is None
    rover = rover or Rover()
    try:
        if angle_initial:
            rover.turn(angle_initial)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

        y = 0.0
        direction = 1  # 1 = row along +x, -1 = row along -x

        while y < y_max:
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            rover.drive(x_max)
            if stop_on_enter.stopped():
                print("Stopped.")
                return

            y += lane_mm
            if y >= y_max:
                break

            # Step one lane in +y; turn direction depends on row heading.
            if direction == 1:
                rover.turn(90)
                if stop_on_enter.stopped():
                    print("Stopped.")
                    return
                rover.drive(lane_mm)
                if stop_on_enter.stopped():
                    print("Stopped.")
                    return
                rover.turn(90)
            else:
                rover.turn(-90)
                if stop_on_enter.stopped():
                    print("Stopped.")
                    return
                rover.drive(lane_mm)
                if stop_on_enter.stopped():
                    print("Stopped.")
                    return
                rover.turn(-90)
            if stop_on_enter.stopped():
                print("Stopped.")
                return
            direction *= -1
    finally:
        if own_rover:
            rover.cleanup()


if __name__ == "__main__":
    stop_on_enter.install()
    basic_search(500, 500, 0)
