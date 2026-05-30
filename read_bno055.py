#!/usr/bin/env python3
"""Observe BNO055 IMU values in a fixed terminal dashboard."""

from __future__ import annotations

import argparse
import time
from collections import deque
from typing import Iterable

import adafruit_bno055
import board


DEFAULT_I2C_ADDRESS = 0x28
HELD_TEMPERATURE_C = -50
HISTORY_LENGTH = 5


class TemperatureReader:
    """Read BNO055 temperature slowly and hold known <= -50 C glitches."""

    def __init__(self, sensor: adafruit_bno055.BNO055, min_interval: float = 1.0) -> None:
        self.sensor = sensor
        self.min_interval = min_interval
        self.last_read_time = 0.0
        self.last_valid: int | None = None
        self.last_raw: int | None = None
        self.last_was_held = False

    def read(self) -> str:
        now = time.monotonic()
        if now - self.last_read_time < self.min_interval:
            return self._format()

        self.last_read_time = now
        raw = self.sensor.temperature
        self.last_raw = raw

        if raw <= HELD_TEMPERATURE_C:
            self.last_was_held = True
        else:
            self.last_valid = raw
            self.last_was_held = False

        return self._format()

    def _format(self) -> str:
        if self.last_valid is None:
            if self.last_raw is None:
                return "not read yet"
            return f"invalid raw {self.last_raw} C"

        if self.last_was_held:
            return f"{self.last_valid} C (held; raw {self.last_raw} C)"

        return f"{self.last_valid} C"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Observe raw and fused values from a BNO055 over Raspberry Pi I2C."
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_I2C_ADDRESS,
        help="I2C address, usually 0x28 or 0x29. Default: 0x28",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Dashboard update rate in Hz. Ignored with --max-rate. Default: 5",
    )
    parser.add_argument(
        "--max-rate",
        action="store_true",
        help="Do not sleep between reads.",
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Read only BNO055 fusion heading/roll/pitch and quaternion.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0 and not args.max_rate:
        raise SystemExit("--rate must be greater than 0")

    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=args.address)
    period = 0.0 if args.max_rate else 1.0 / args.rate

    print("\033[2J", end="")

    try:
        if args.fast:
            run_fast_loop(sensor, args.address, period)
        else:
            run_full_loop(sensor, args.address, period)
    except KeyboardInterrupt:
        print("\033[H", end="")
        print("Stopped.")


def run_fast_loop(sensor: adafruit_bno055.BNO055, address: int, period: float) -> None:
    history: deque[dict[str, object]] = deque(maxlen=HISTORY_LENGTH)
    previous_loop: float | None = None

    while True:
        loop_start = time.monotonic()
        loop_hz = hz_since(previous_loop, loop_start)
        previous_loop = loop_start

        row = {
            "t": loop_start,
            "hz": loop_hz,
            "euler": sensor.euler,
            "quaternion": sensor.quaternion,
        }
        history.appendleft(row)

        draw_fast_dashboard(address, history)
        sleep_until_next_frame(loop_start, period)


def run_full_loop(sensor: adafruit_bno055.BNO055, address: int, period: float) -> None:
    history: deque[dict[str, object]] = deque(maxlen=HISTORY_LENGTH)
    temperature = TemperatureReader(sensor)
    previous_loop: float | None = None

    while True:
        loop_start = time.monotonic()
        loop_hz = hz_since(previous_loop, loop_start)
        previous_loop = loop_start
        sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status

        row = {
            "t": loop_start,
            "hz": loop_hz,
            "calibration": f"{sys_cal}/{gyro_cal}/{accel_cal}/{mag_cal}",
            "temperature": temperature.read(),
            "accel": sensor.acceleration,
            "gyro": sensor.gyro,
            "mag": sensor.magnetic,
            "euler": sensor.euler,
            "quaternion": sensor.quaternion,
            "linear": sensor.linear_acceleration,
            "gravity": sensor.gravity,
        }
        history.appendleft(row)

        draw_full_dashboard(address, history)
        sleep_until_next_frame(loop_start, period)


def draw_fast_dashboard(address: int, history: deque[dict[str, object]]) -> None:
    print("\033[H", end="")
    dashboard_line(f"BNO055 dashboard | I2C 0x{address:02x} | mode fast | Ctrl-C to stop")
    dashboard_line("t        hz    heading  roll   pitch      qw      qx      qy      qz")
    dashboard_line("-----------------------------------------------------------------------")
    for row in history:
        heading, roll, pitch = vector_values(row["euler"], 3)
        qw, qx, qy, qz = vector_values(row["quaternion"], 4)
        dashboard_line(
            f"{row['t']:8.3f} {row['hz']:5.1f} "
            f"{heading:8.2f} {roll:6.2f} {pitch:7.2f} "
            f"{qw:7.4f} {qx:7.4f} {qy:7.4f} {qz:7.4f}"
        )
    clear_remaining_lines(HISTORY_LENGTH - len(history))


def draw_full_dashboard(address: int, history: deque[dict[str, object]]) -> None:
    print("\033[H", end="")
    dashboard_line(f"BNO055 dashboard | I2C 0x{address:02x} | mode full | Ctrl-C to stop")
    dashboard_line("t        hz    cal     temp                      accel xyz                  gyro xyz                   mag xyz")
    dashboard_line("----------------------------------------------------------------------------------------------------------------")
    for row in history:
        dashboard_line(
            f"{row['t']:8.3f} {row['hz']:5.1f} {row['calibration']:>7} "
            f"{str(row['temperature']):24.24} "
            f"{fmt_vector(row['accel'])} {fmt_vector(row['gyro'])} {fmt_vector(row['mag'])}"
        )

    dashboard_line()
    dashboard_line("t        hz    heading  roll   pitch      qw      qx      qy      qz       linear xyz                gravity xyz")
    dashboard_line("----------------------------------------------------------------------------------------------------------------")
    for row in history:
        heading, roll, pitch = vector_values(row["euler"], 3)
        qw, qx, qy, qz = vector_values(row["quaternion"], 4)
        dashboard_line(
            f"{row['t']:8.3f} {row['hz']:5.1f} "
            f"{heading:8.2f} {roll:6.2f} {pitch:7.2f} "
            f"{qw:7.4f} {qx:7.4f} {qy:7.4f} {qz:7.4f} "
            f"{fmt_vector(row['linear'])} {fmt_vector(row['gravity'])}"
        )
    clear_remaining_lines((HISTORY_LENGTH - len(history)) * 2)


def hz_since(previous: float | None, current: float) -> float:
    if previous is None or current == previous:
        return 0.0
    return 1.0 / (current - previous)


def sleep_until_next_frame(loop_start: float, period: float) -> None:
    if period <= 0:
        return

    sleep_time = period - (time.monotonic() - loop_start)
    if sleep_time > 0:
        time.sleep(sleep_time)


def vector_values(values: object, length: int) -> tuple[float, ...]:
    if not isinstance(values, Iterable):
        return (float("nan"),) * length

    result = []
    for value in values:
        result.append(float("nan") if value is None else float(value))

    while len(result) < length:
        result.append(float("nan"))
    return tuple(result[:length])


def fmt_vector(values: object) -> str:
    return "(" + ", ".join(f"{value:7.3f}" for value in vector_values(values, 3)) + ")"


def clear_remaining_lines(count: int) -> None:
    for _ in range(max(0, count)):
        dashboard_line()


def dashboard_line(text: str = "") -> None:
    print(f"{text}\033[K")


if __name__ == "__main__":
    main()
