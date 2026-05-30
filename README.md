# BNO055 IMU quick observer

This directory is a small Raspberry Pi Zero 2 W test setup for a GY-BNO055/BNO055 IMU.

## Setup

The virtual environment has already been created with `uv`, and the Adafruit BNO055 library was cloned into:

```bash
vendor/Adafruit_CircuitPython_BNO055
```

Installed Python packages include `adafruit-blinka`, the editable BNO055 library clone, and `RPi.GPIO`.

Activate the environment:

```bash
source .venv/bin/activate
```

## Wiring for I2C

- Pi 3V3 -> BNO055 VCC/VIN
- Pi GND -> BNO055 GND
- Pi GPIO2/SDA, physical pin 3 -> BNO055 SDA
- Pi GPIO3/SCL, physical pin 5 -> BNO055 SCL

Check that the board is visible:

```bash
i2cdetect -y 1
```

Expected address is usually `0x28` or `0x29`. This board was detected at `0x29`.

## Run

Default address `0x28`:

```bash
uv run python read_bno055.py
```

If `i2cdetect` shows `0x29`:

```bash
uv run python read_bno055.py --address 0x29
```

Fast dashboard with only BNO055 fusion heading/roll/pitch and quaternion:

```bash
sudo .venv/bin/python read_bno055.py --address 0x29 --fast --rate 50
```

Full dashboard with raw and fusion values:

```bash
sudo .venv/bin/python read_bno055.py --address 0x29 --rate 5
```

Run without sleeping between reads:

```bash
sudo .venv/bin/python read_bno055.py --address 0x29 --fast --max-rate
```
