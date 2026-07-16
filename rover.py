"""Rover drive & turn primitives for a 4-motor Raspberry Pi rover.

This module exposes two high-level, encoder-closed-loop motions that the
radiation-search algorithms will call:

    rover.drive(distance_mm)   # straight line travel by distance
    rover.turn(angle_deg)      # spin in place by a heading change

Hardware:
    - Adafruit DC Motor HAT (I2C), motors addressed kit.motor1..4
    - 4x DFRobot FIT0522 gearmotors (12 CPR encoder, 75:1 gearbox)
    - Quadrature encoders read via RPi.GPIO interrupts

"""

import math
import time

import board  # pyright: ignore[reportMissingImports]
import RPi.GPIO as GPIO  # pyright: ignore[reportMissingImports, reportMissingModuleSource]
from adafruit_motorkit import MotorKit  # pyright: ignore[reportMissingImports]


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
#   1 = front-right   2 = front-left   3 = rear-left   4 = rear-right
# Front motors were wired opposite the rears on this build, so FL/FR are flipped.
MOTOR_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}

# Encoder GPIO pins (BCM numbering): motor -> (channel A, channel B).
# Layout is optimized for a T-Cobbler: all 8 signals sit in the bottom pin
# block (physical 29–40), A/B pairs are adjacent, and I2C (GPIO2/3 for the
# Motor HAT) is left free at the top of the header.
#
#   Motor   Role          Chan A   Chan B   Cobbler labels   Phys pins
#   -----   ----          ------   ------   --------------   ---------
#     1     front-right     5        6      GPIO5 / GPIO6    29 / 31
#     2     front-left     13       19      GPIO13 / GPIO19  33 / 35
#     3     rear-left      26       20      GPIO26 / GPIO20  37 / 38
#     4     rear-right     16       21      GPIO16 / GPIO21  36 / 40
#
# Also share GND with the Pi (e.g. phys 30/34/39) and power encoders from
# 3.3V (phys 1 or 17) unless your encoder boards are already level-shifted.
ENC_PINS = {
    1: (5, 6),
    2: (13, 19),
    3: (26, 20),
    4: (16, 21),
}

# Physical sides on this build: motor channels 1 and 2 are swapped.
LEFT_MOTORS = (2, 3)
RIGHT_MOTORS = (1, 4)

# Default throttles -- modest so the rover is controllable; callers can override.
DEFAULT_DRIVE_THROTTLE = 0.5
DEFAULT_TURN_THROTTLE = 0.3

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

        # Encoder counts and last-A state, per motor.
        self.counts = {1: 0, 2: 0, 3: 0, 4: 0}

        self._setup_encoders()
        self.stop()

    # ------------------------------------------------------------------
    # Encoder layer
    # ------------------------------------------------------------------
    def _setup_encoders(self):
        GPIO.setmode(GPIO.BCM)
        for name, (pin_a, pin_b) in ENC_PINS.items():
            GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            cb = self._make_callback(name, pin_a, pin_b)
            # GPIO.BOTH on both channels gives full x4 quadrature decoding.
            GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=cb)
            GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=cb)

    def _make_callback(self, name, pin_a, pin_b):
        def callback(channel):
            a = GPIO.input(pin_a)
            b = GPIO.input(pin_b)
            if channel == pin_a:
                # A edge: direction set by whether A and B now differ.
                self.counts[name] += 1 if a != b else -1
            else:
                # B edge: direction set by whether A and B now match.
                self.counts[name] += 1 if a == b else -1
        return callback

    def _reset_counts(self):
        for name in self.counts:
            self.counts[name] = 0

    def _avg_abs_counts(self):
        """Mean absolute count across the four wheels (robust to one bad encoder)."""
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
    def _set_logical(self, fl, fr, rl, rr):
        """Set per-wheel throttles in plain forward(+)/backward(-) terms.

        Mirroring is applied here via MOTOR_SIGN so callers never deal with it.
        """
        logical = {1: fl, 2: fr, 3: rl, 4: rr}
        for name, value in logical.items():
            self._motors[name].throttle = MOTOR_SIGN[name] * value

    def _drive_sides(self, left, right):
        """Drive the left wheels (2,3) and right wheels (1,4) at logical throttles."""
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
                time.sleep(0.005)
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def cleanup(self):
        """Stop motors and release GPIO. Call once at program exit."""
        self.stop()
        GPIO.cleanup()


def _demo():
    """Bench demo: drive forward 500 mm, then turn left 90 degrees."""
    rover = Rover()
    try:
        print("Driving forward 500 mm...")
        rover.drive(500)
        time.sleep(0.5)

        print("Turning left 90 degrees...")
        rover.turn(90)
        time.sleep(0.5)

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

        y = 0.0
        direction = 1  # 1 = row along +x, -1 = row along -x

        while y < y_max:
            rover.drive(x_max)

            y += lane_mm
            if y >= y_max:
                break

            # Step one lane in +y; turn direction depends on row heading.
            if direction == 1:
                rover.turn(90)
                rover.drive(lane_mm)
                rover.turn(90)
            else:
                rover.turn(-90)
                rover.drive(lane_mm)
                rover.turn(-90)
            direction *= -1
    finally:
        if own_rover:
            rover.cleanup()


if __name__ == "__main__":
    basic_search(500, 500, 0)
