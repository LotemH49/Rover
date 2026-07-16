# Rover

Encoder-closed-loop drive and turn primitives for a 4-motor Raspberry Pi rover, plus a browser-based simulator for development on a laptop.

## Clone on Linux

Install git if needed:

```bash
sudo apt update && sudo apt install git
```

Clone the repo (HTTPS):

```bash
git clone https://github.com/LotemH49/Rover.git
cd Rover
```

Or with SSH (if you have a GitHub SSH key set up):

```bash
git clone git@github.com:LotemH49/Rover.git
cd Rover
```

## Requirements

Modern Linux / Raspberry Pi OS blocks system-wide `pip install` (externally-managed-environment). Use a virtual environment.

On a **Pi 5**, install system `lgpio` first (pip cannot build it without extra tools), then create the venv with access to system packages:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3-lgpio
cd ~/Rover
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 startup_test.py
```

If you already have a venv **without** system site packages, recreate it:

```bash
deactivate
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Activate the venv again in future sessions: `source .venv/bin/activate`

If you must install without a venv (not recommended):

```bash
pip3 install --break-system-packages -r requirements.txt
```
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

Enable I2C (`sudo raspi-config` → Interface Options → I2C), then use the [Requirements](#requirements) venv steps above.

## API

```python
from rover import Rover

rover = Rover()
rover.drive(500)   # mm forward
rover.turn(90)     # degrees CCW
rover.cleanup()
```

`basic_search(x_max, y_max, angle_initial)` runs a lawnmower grid search over a rectangle.
