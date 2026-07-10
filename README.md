# Rover

Encoder-closed-loop drive and turn primitives for a 4-motor Raspberry Pi rover, plus a browser-based simulator for development on a laptop.

## Hardware

- Adafruit DC Motor HAT (I2C)
- 4× DFRobot FIT0522 gearmotors (12 CPR encoder, 75:1 gearbox)
- Quadrature encoders on GPIO

## Simulator (laptop)

No extra dependencies — uses the standard library only.

```bash
python3 simulator.py
```

Open http://localhost:8000 to drive the rover, run the demo, or watch the lawnmower search sweep a 500×500 mm area.

## Raspberry Pi

Enable I2C (`sudo raspi-config` → Interface Options → I2C), then:

```bash
pip3 install -r requirements.txt
python3 rover.py
```

## API

```python
from rover import Rover

rover = Rover()
rover.drive(500)   # mm forward
rover.turn(90)     # degrees CCW
rover.cleanup()
```

`basic_search(x_max, y_max, angle_initial)` runs a lawnmower grid search over a rectangle.
