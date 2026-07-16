"""Fake Raspberry Pi hardware + physics engine for the rover simulator.

This lets the *unmodified* ``rover.py`` run on a laptop. It installs stand-in
modules for ``board``, ``RPi.GPIO`` and ``adafruit_motorkit`` into
``sys.modules`` so that ``import``s inside ``rover.py`` resolve to these fakes.

The physics is a standard differential-drive model:
    - each motor's throttle -> a wheel forward speed
    - left wheels (1,3) and right wheels (2,4) average into v_left / v_right
    - body linear/angular velocity integrate the (x, y, theta) pose
    - each wheel's travel generates *real* quadrature encoder edges, which we
      feed back into rover.py's GPIO callbacks exactly like the hardware would.

Constants below MUST match rover.py so the simulated distances line up with the
code's encoder math.
"""

import math
import sys
import threading
import time
import types

# --- Must match rover.py -------------------------------------------------
COUNTS_PER_REV = 900
WHEEL_DIAMETER_MM = 65.0
WHEEL_CIRC_MM = math.pi * WHEEL_DIAMETER_MM
TRACK_WIDTH_MM = 331.8

# How the wheels are physically wired/mounted. With the right side mirrored,
# a forward command (+left / -right throttle from rover.MOTOR_SIGN) must move
# the whole rover forward, so the sim's mount sign mirrors that mapping.
#   1 = front-right  2 = front-left  3 = rear-left  4 = rear-right
MOUNT_SIGN = {1: -1, 2: +1, 3: +1, 4: -1}

# Wheel speed (mm/s) at throttle = 1.0. Tune to taste for the sim.
MAX_WHEEL_SPEED_MM_S = 260.0

LEFT_MOTORS = (2, 3)
RIGHT_MOTORS = (1, 4)


# =========================================================================
# Fake GPIO
# =========================================================================
class _FakeGPIO:
    BCM = "BCM"
    BOARD = "BOARD"
    IN = "IN"
    OUT = "OUT"
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    BOTH = "BOTH"
    RISING = "RISING"
    FALLING = "FALLING"

    def __init__(self):
        self._levels = {}
        self._callbacks = {}

    def setmode(self, mode):
        pass

    def setwarnings(self, flag):
        pass

    def setup(self, pin, mode, pull_up_down=None):
        self._levels.setdefault(pin, 0)

    def add_event_detect(self, pin, edge, callback=None, bouncetime=None):
        if callback is not None:
            self._callbacks[pin] = callback

    def remove_event_detect(self, pin):
        self._callbacks.pop(pin, None)

    def input(self, pin):
        return self._levels.get(pin, 0)

    def output(self, pin, value):
        self._levels[pin] = value

    def cleanup(self, pin=None):
        self._callbacks.clear()


# =========================================================================
# Physics engine
# =========================================================================
class Simulator:
    def __init__(self, gpio):
        self.gpio = gpio

        # Throttles written by the fake motors (-1..1, or None for coast).
        self.throttles = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        # Pose in mm / radians. theta = 0 points along +x; +theta is CCW.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder state per wheel: quadrature step q + fractional accumulator.
        # Filled in once rover.py registers its encoder pins.
        self.enc = {}  # idx -> {"a": pinA, "b": pinB, "q": 0, "acc": 0.0}

        self.trail = [(0.0, 0.0)]
        self._lock = threading.Lock()
        self.running = False
        self._thread = None

    # -- encoder pin registration (called from fake GPIO via rover setup) --
    def bind_encoder(self, idx, pin_a, pin_b):
        self.enc[idx] = {"a": pin_a, "b": pin_b, "q": 0, "acc": 0.0}

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False

    def reset(self):
        with self._lock:
            self.x = self.y = self.theta = 0.0
            self.trail = [(0.0, 0.0)]
            for e in self.enc.values():
                e["q"] = 0
                e["acc"] = 0.0

    # -- read-only snapshot for the UI ------------------------------------
    def snapshot(self):
        with self._lock:
            return {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "throttles": dict(self.throttles),
                "trail": list(self.trail[-2000:]),
            }

    # -- core integration --------------------------------------------------
    def _loop(self):
        last = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = now - last
            last = now
            if dt > 0:
                self._step(dt)
            time.sleep(0.005)

    def _wheel_speed(self, idx):
        t = self.throttles.get(idx)
        if t is None:
            return 0.0
        return t * MOUNT_SIGN[idx] * MAX_WHEEL_SPEED_MM_S

    def _step(self, dt):
        s1 = self._wheel_speed(1)
        s2 = self._wheel_speed(2)
        s3 = self._wheel_speed(3)
        s4 = self._wheel_speed(4)

        # Physical sides on this build: motor channels 1 and 2 are swapped.
        v_left = (s2 + s3) / 2.0
        v_right = (s1 + s4) / 2.0
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / TRACK_WIDTH_MM

        with self._lock:
            self.theta += omega * dt
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            last_x, last_y = self.trail[-1]
            if (self.x - last_x) ** 2 + (self.y - last_y) ** 2 > 25.0:  # >5mm
                self.trail.append((self.x, self.y))
                if len(self.trail) > 6000:
                    self.trail = self.trail[-6000:]

        # Generate encoder edges per wheel (outside the pose lock so callbacks
        # in rover.py run just like real interrupts).
        speeds = {1: s1, 2: s2, 3: s3, 4: s4}
        for idx, speed in speeds.items():
            e = self.enc.get(idx)
            if e is None:
                continue
            dcounts = (speed * dt) / WHEEL_CIRC_MM * COUNTS_PER_REV
            self._advance_encoder(e, dcounts)

    def _advance_encoder(self, e, dcounts):
        e["acc"] += dcounts
        # Guard against runaway loops if dt ever spikes.
        max_steps = 5000
        while e["acc"] >= 1.0 and max_steps > 0:
            self._enc_step(e, +1)
            e["acc"] -= 1.0
            max_steps -= 1
        while e["acc"] <= -1.0 and max_steps > 0:
            self._enc_step(e, -1)
            e["acc"] += 1.0
            max_steps -= 1

    def _enc_step(self, e, direction):
        old = e["q"]
        new = old + direction
        e["q"] = new

        oa = 1 if old % 4 in (1, 2) else 0
        na = 1 if new % 4 in (1, 2) else 0
        nb = 1 if new % 4 in (2, 3) else 0

        self.gpio._levels[e["a"]] = na
        self.gpio._levels[e["b"]] = nb

        channel = e["a"] if na != oa else e["b"]
        cb = self.gpio._callbacks.get(channel)
        if cb is not None:
            cb(channel)


# =========================================================================
# Fake MotorKit
# =========================================================================
class _FakeMotor:
    def __init__(self, sim, idx):
        self._sim = sim
        self._idx = idx
        self._throttle = 0.0

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = value
        self._sim.throttles[self._idx] = value


class _FakeMotorKit:
    def __init__(self, i2c=None, address=None):
        self.motor1 = _FakeMotor(SIM, 1)
        self.motor2 = _FakeMotor(SIM, 2)
        self.motor3 = _FakeMotor(SIM, 3)
        self.motor4 = _FakeMotor(SIM, 4)


# =========================================================================
# Module-level singletons + installation
# =========================================================================
GPIO = _FakeGPIO()
SIM = Simulator(GPIO)

# Wrap the real setup call so we learn the encoder pin pairs as rover.py
# registers them, then bind them to the physics engine.
_orig_add_event_detect = GPIO.add_event_detect


def _tracking_add_event_detect(pin, edge, callback=None, bouncetime=None):
    _orig_add_event_detect(pin, edge, callback=callback, bouncetime=bouncetime)


GPIO.add_event_detect = _tracking_add_event_detect


def install():
    """Register the fake modules so ``import``s in rover.py resolve to them."""
    board_mod = types.ModuleType("board")
    board_mod.I2C = lambda: object()

    motorkit_mod = types.ModuleType("adafruit_motorkit")
    motorkit_mod.MotorKit = _FakeMotorKit

    rpi_pkg = types.ModuleType("RPi")
    rpi_pkg.GPIO = GPIO
    gpio_mod = GPIO  # an object with the needed attributes is sufficient

    sys.modules["board"] = board_mod
    sys.modules["adafruit_motorkit"] = motorkit_mod
    sys.modules["RPi"] = rpi_pkg
    sys.modules["RPi.GPIO"] = gpio_mod

    return SIM


def bind_encoders_from(enc_pins):
    """Bind encoder pin pairs (motor -> (A, B)) to the physics engine."""
    for idx, (pin_a, pin_b) in enc_pins.items():
        SIM.bind_encoder(idx, pin_a, pin_b)
