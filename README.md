# BNO055 IMU quick observer

This directory is a small Raspberry Pi Zero 2 W test setup for a GY-BNO055/BNO055 IMU.

> **자세 추정 알고리즘 직접 구현 실습은 [`README_filters.md`](README_filters.md) 참고.**
> BNO055 를 AMG(raw) 모드로 두고 경사계 → 자이로 적분 → 상보 필터 → BNO055 정답(NDOF)
> 비교를 4단계로 직접 구현했습니다. `smbus2` 만 쓰고 adafruit/numpy/matplotlib 불필요
> (CLI 전용). 실측상 내 상보필터가 내장 퓨전을 **0.11° 오차**로 따라갑니다.

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

## Experimental web dashboard

This is a prototype for the dashboard idea. On a Raspberry Pi Zero 2 W, the
browser and Three.js view can be heavy, so prefer the terminal dashboard for
quick and reliable IMU debugging.

Start the browser dashboard:

```bash
sudo .venv/bin/python web_dashboard.py --address 0x29 --host 0.0.0.0 --port 1190
```

Open this machine's browser at:

```text
http://127.0.0.1:1190
```

Or open it from another machine on the same network with the Raspberry Pi IP:

```text
http://<raspberry-pi-ip>:1190
```

The web dashboard includes:

- Three.js 3D orientation view driven by the BNO055 quaternion
- Euler/quaternion/gravity/raw value checkboxes
- 10-100 Hz stream rate slider
- Quaternion norm, gravity magnitude, calibration, and observed rate health checks
