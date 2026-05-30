#!/usr/bin/env python3
"""Web dashboard for observing a BNO055 IMU from a Raspberry Pi."""

from __future__ import annotations

import argparse
import asyncio
import math
import time
from pathlib import Path
from typing import Any

import adafruit_bno055
import board
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketState


DEFAULT_I2C_ADDRESS = 0x28
DEFAULT_RATE_HZ = 50
MIN_RATE_HZ = 1
MAX_RATE_HZ = 100
DEFAULT_PORT = 1190
MIN_PORT = 1190
MAX_PORT = 2000
STATIC_DIR = Path(__file__).parent / "static"


app = FastAPI()
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve a BNO055 web dashboard.")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_I2C_ADDRESS,
        help="I2C address, usually 0x28 or 0x29. Default: 0x28",
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP host. Default: 0.0.0.0")
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"HTTP port, {MIN_PORT}-{MAX_PORT}. Default: {DEFAULT_PORT}",
    )
    return parser.parse_args()


class DashboardConfig:
    def __init__(self) -> None:
        self.rate_hz = DEFAULT_RATE_HZ
        self.fields = {
            "euler": True,
            "quaternion": True,
            "gravity": True,
            "linear": False,
            "accel": False,
            "gyro": False,
            "magnetometer": False,
        }

    def update(self, message: dict[str, Any]) -> None:
        if "rate_hz" in message:
            self.rate_hz = clamp_rate(message["rate_hz"])

        fields = message.get("fields")
        if isinstance(fields, dict):
            for key in self.fields:
                if key in fields:
                    self.fields[key] = bool(fields[key])


class BNO055Reader:
    def __init__(self, address: int) -> None:
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)
        self.address = address
        self.previous_read: float | None = None
        self.status_read_time = 0.0
        self.status = {
            "calibration": [0, 0, 0, 0],
            "temperature_c": None,
        }

    def read(self, fields: dict[str, bool]) -> dict[str, Any]:
        now = time.monotonic()
        hz = 0.0 if self.previous_read is None else 1.0 / (now - self.previous_read)
        self.previous_read = now

        data: dict[str, Any] = {
            "t": now,
            "hz": hz,
            "address": self.address,
            "values": {},
            "health": {},
        }

        # Keep orientation available for the 3D view even if a checkbox is off.
        euler = self.sensor.euler
        quaternion = self.sensor.quaternion
        gravity = self.sensor.gravity

        if fields.get("euler", True):
            data["values"]["euler"] = as_list(euler)
        if fields.get("quaternion", True):
            data["values"]["quaternion"] = as_list(quaternion)
        if fields.get("gravity", True):
            data["values"]["gravity"] = as_list(gravity)
        if fields.get("linear"):
            data["values"]["linear"] = as_list(self.sensor.linear_acceleration)
        if fields.get("accel"):
            data["values"]["accel"] = as_list(self.sensor.acceleration)
        if fields.get("gyro"):
            data["values"]["gyro"] = as_list(self.sensor.gyro)
        if fields.get("magnetometer"):
            data["values"]["magnetometer"] = as_list(self.sensor.magnetic)

        data["orientation"] = {
            "euler": as_list(euler),
            "quaternion": as_list(quaternion),
            "gravity": as_list(gravity),
        }
        data["health"] = self.health(now, quaternion, gravity, hz)
        return data

    def health(
        self,
        now: float,
        quaternion: tuple[float | None, ...],
        gravity: tuple[float | None, ...],
        hz: float,
    ) -> dict[str, Any]:
        if now - self.status_read_time >= 1.0:
            self.status_read_time = now
            sys_cal, gyro_cal, accel_cal, mag_cal = self.sensor.calibration_status
            self.status = {
                "calibration": [sys_cal, gyro_cal, accel_cal, mag_cal],
                "temperature_c": self.sensor.temperature,
            }

        quat_norm = vector_norm(quaternion)
        gravity_mag = vector_norm(gravity)
        return {
            **self.status,
            "quat_norm": quat_norm,
            "quat_ok": quat_norm is not None and 0.95 <= quat_norm <= 1.05,
            "gravity_mag": gravity_mag,
            "gravity_ok": gravity_mag is not None and 8.5 <= gravity_mag <= 11.0,
            "rate_hz": hz,
        }


config = DashboardConfig()
reader: BNO055Reader | None = None


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/state")
async def api_state() -> dict[str, Any]:
    return {"rate_hz": config.rate_hz, "fields": config.fields}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    assert reader is not None
    receiver_task = asyncio.create_task(receive_config_loop(websocket))

    try:
        while True:
            if websocket.client_state != WebSocketState.CONNECTED:
                return

            loop_start = time.monotonic()
            sample = reader.read(config.fields)
            sample["config"] = {"rate_hz": config.rate_hz, "fields": config.fields}
            await websocket.send_json(sample)

            sleep_time = (1.0 / config.rate_hz) - (time.monotonic() - loop_start)
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                await asyncio.sleep(0)
    except WebSocketDisconnect:
        return
    except RuntimeError:
        return
    finally:
        receiver_task.cancel()


async def receive_config_loop(websocket: WebSocket) -> None:
    while True:
        message = await websocket.receive_json()
        config.update(message)


def clamp_rate(value: Any) -> int:
    try:
        rate = int(value)
    except (TypeError, ValueError):
        return DEFAULT_RATE_HZ
    return max(MIN_RATE_HZ, min(MAX_RATE_HZ, rate))


def as_list(values: Any) -> list[float | None]:
    if values is None:
        return []
    return [None if value is None else float(value) for value in values]


def vector_norm(values: Any) -> float | None:
    if values is None or any(value is None for value in values):
        return None
    return math.sqrt(sum(float(value) * float(value) for value in values))


def main() -> None:
    global reader

    args = parse_args()
    if not MIN_PORT <= args.port <= MAX_PORT:
        raise SystemExit(f"--port must be between {MIN_PORT} and {MAX_PORT}")

    reader = BNO055Reader(args.address)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
