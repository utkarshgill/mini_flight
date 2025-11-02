#!/usr/bin/env python3
"""Launch the browser-based configurator to visualize live IMU telemetry."""

from __future__ import annotations

import os
import struct
import sys
import time
import threading
from pathlib import Path
from typing import Generator, Iterable, Optional

from common.serve import RendererServer, SharedState, maybe_launch_browser
from miniflight.filters import ComplementaryFilter, Filter

try:
    import termios
except ImportError:  # pragma: no cover - Windows fallback
    termios = None  # type: ignore


DEVICE = os.environ.get("MINIFLIGHT_SERIAL", "/dev/tty.usbmodem0x80000001")
BAUD = int(os.environ.get("MINIFLIGHT_SERIAL_BAUD", "115200"))
CMD_RAW_IMU = 102
ACC_LSB_PER_G = 2048.0
GYRO_LSB_PER_DPS = 16.384


def xor_checksum(data: Iterable[int]) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


def msp_request(cmd: int) -> bytes:
    size = 0
    checksum = size ^ cmd
    return b"$M<" + bytes([size, cmd, checksum])


def parse_buffer(buf: bytearray) -> Generator[bytes, None, None]:
    while True:
        start = buf.find(b"$M>")
        if start == -1:
            buf.clear()
            break
        if len(buf) < start + 5:
            break
        size = buf[start + 3]
        frame_len = 5 + size + 1
        if len(buf) < start + frame_len:
            break
        frame = bytes(buf[start:start + frame_len])
        del buf[:start + frame_len]
        yield frame


def open_serial():
    if termios is None:
        raise RuntimeError("Serial support unavailable on this platform")
    fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    baud_const = getattr(termios, f"B{BAUD}", None)
    if hasattr(termios, "cfsetispeed"):
        termios.cfsetispeed(attrs, baud_const)
        termios.cfsetospeed(attrs, baud_const)
    else:
        attrs[4] = baud_const
        attrs[5] = baud_const
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_frames(fd: int) -> Generator[bytes, None, None]:
    buf = bytearray()
    while True:
        os.write(fd, msp_request(CMD_RAW_IMU))
        time.sleep(0.02)
        try:
            buf.extend(os.read(fd, 512))
        except BlockingIOError:
            continue
        yield from parse_buffer(buf)


def decode_raw_imu(frame: bytes) -> tuple[int, ...]:
    size = frame[3]
    payload = frame[5:5 + size]
    checksum = frame[5 + size]
    calc = xor_checksum(frame[3:5 + size])
    if calc != checksum:
        raise ValueError(f"Checksum mismatch: {calc} != {checksum}")
    return struct.unpack_from("<9h", payload)  # ax, ay, az, gx, gy, gz, mx, my, mz


def measure_bias(frames: Generator[bytes, None, None], samples: int = 200) -> tuple[list[float], list[float]]:
    acc_sum = [0.0, 0.0, 0.0]
    gyro_sum = [0.0, 0.0, 0.0]
    for _ in range(samples):
        raw = decode_raw_imu(next(frames))
        ax, ay, az, gx, gy, gz, *_ = raw
        acc_sum[0] += ax
        acc_sum[1] += ay
        acc_sum[2] += az
        gyro_sum[0] += gx
        gyro_sum[1] += gy
        gyro_sum[2] += gz
    acc_bias = [s / samples for s in acc_sum]
    gyro_bias = [s / samples for s in gyro_sum]
    return acc_bias, gyro_bias


def apply_bias(values: tuple[int, ...], acc_bias, gyro_bias) -> tuple[float, float, float, float, float, float]:
    ax, ay, az, gx, gy, gz, *_ = values
    ax = (ax - acc_bias[0]) / ACC_LSB_PER_G
    ay = (ay - acc_bias[1]) / ACC_LSB_PER_G
    az = (az - acc_bias[2]) / ACC_LSB_PER_G
    gx = (gx - gyro_bias[0]) / GYRO_LSB_PER_DPS
    gy = (gy - gyro_bias[1]) / GYRO_LSB_PER_DPS
    gz = (gz - gyro_bias[2]) / GYRO_LSB_PER_DPS
    return ax, ay, az, gx, gy, gz


def imu_stream(shared: SharedState, orientation_filter: Optional[Filter] = None) -> None:
    fd = open_serial()
    print(f"Connected to {DEVICE} @ {BAUD} baud")
    frames = read_frames(fd)
    print("Hold the craft still while we measure bias…")
    acc_bias, gyro_bias = measure_bias(frames)
    acc_fmt = ", ".join(f"{v:.1f}" for v in acc_bias)
    gyro_fmt = ", ".join(f"{v:.1f}" for v in gyro_bias)
    print(f"Bias calibrated: accel=[{acc_fmt}] gyro=[{gyro_fmt}]")
    try:
        start = time.perf_counter()
        last_update = None
        for frame in frames:
            raw = decode_raw_imu(frame)
            ax, ay, az, gx, gy, gz = apply_bias(raw, acc_bias, gyro_bias)
            loop_time = time.perf_counter()
            world_time = loop_time - start
            dt = 0.0 if last_update is None else loop_time - last_update
            last_update = loop_time

            attitude = None
            if orientation_filter is not None:
                attitude = orientation_filter.update((ax, ay, az), (gx, gy, gz), dt)

            snapshot = {
                "world_time": world_time,
                "imu": {
                    "accel": [ax, ay, az],
                    "gyro": [gx, gy, gz],
                    "source": DEVICE,
                },
            }
            if attitude:
                snapshot["attitude"] = attitude

            shared.set_snapshot(snapshot)
            time.sleep(0.02)
    finally:
        if fd is not None:
            os.close(fd)


def main() -> None:
    static_dir = Path(__file__).resolve().parent
    shared_state = SharedState()
    attitude_filter = ComplementaryFilter()
    host = os.getenv("MINIFLIGHT_CONFIG_HOST", os.getenv("MINIFLIGHT_RENDER_HOST", "127.0.0.1"))
    port = int(os.getenv("MINIFLIGHT_CONFIG_PORT", "8002"))
    renderer = RendererServer(shared_state, host=host, port=port, static_dir=static_dir)
    renderer.start()
    shared_state.set_snapshot({
        "world_time": 0.0,
        "imu": {
            "accel": [0, 0, 0],
            "gyro": [0, 0, 0],
            "source": "waiting",
        },
        "attitude": {
            "filter": attitude_filter.name,
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "euler_deg": [0.0, 0.0, 0.0],
            "euler_rad": [0.0, 0.0, 0.0],
        },
    })
    url = f"http://{host}:{port}".replace("0.0.0.0", "127.0.0.1")
    print(f"*** Configurator running at {url}")
    maybe_launch_browser(url)
    producer = threading.Thread(target=imu_stream, args=(shared_state, attitude_filter), daemon=True)
    producer.start()
    try:
        while producer.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping…")
    finally:
        renderer.shutdown()


if __name__ == "__main__":
    if not hasattr(sys, "excepthook"):
        sys.excepthook = lambda exc_type, exc, tb: print(exc, file=sys.stderr)
    main()

