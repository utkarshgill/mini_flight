"""
Command sources for pilot input (keyboard, DualSense).
These map physical inputs to high-level PilotCommand used by firmware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

import math

from common.interface import CommandSource
from common.types import PilotCommand


class KeyboardCommandSource(CommandSource):
    """
    Produces PilotCommand from a key-state provider.
    Keys: W/S adjust altitude setpoint up/down; arrows control roll/pitch; A/D yaw.
    """

    def __init__(
        self,
        get_keys: Callable[[], Dict[str, bool]],
        max_tilt: float = 0.5,  # rad
        yaw_rate_gain: float = math.pi / 2,  # rad/s at full deflection
        z_gain: float = 5.0,  # m/s change for W/S (integrated via dt)
    ):
        self._get_keys = get_keys
        self._max_tilt = float(max_tilt)
        self._yaw_rate_gain = float(yaw_rate_gain)
        self._z_gain = float(z_gain)

    def read(self, dt: float) -> PilotCommand:
        keys = self._get_keys() or {}
        z_delta = 0.0
        if keys.get("w", False):
            z_delta += self._z_gain * dt
        if keys.get("s", False):
            z_delta -= self._z_gain * dt

        roll_t = (
            -self._max_tilt if keys.get("left", False) else self._max_tilt if keys.get("right", False) else 0.0
        )
        pitch_t = (
            self._max_tilt if keys.get("up", False) else -self._max_tilt if keys.get("down", False) else 0.0
        )
        yaw_rate = 0.0
        if keys.get("a", False):
            yaw_rate += self._yaw_rate_gain
        if keys.get("d", False):
            yaw_rate -= self._yaw_rate_gain
        return PilotCommand(roll_target=roll_t, pitch_target=pitch_t, yaw_rate=yaw_rate, z_setpoint_delta=z_delta)

    def close(self) -> None:
        return None


class DualSenseCommandSource(CommandSource):
    """PS5 DualSense gamepad via hidapi."""

    VENDOR_ID = 0x054C
    PRODUCT_ID = 0x0CE6

    def __init__(
        self,
        max_tilt: float = 0.5,
        yaw_rate_gain: float = math.pi / 2,
        z_gain: float = 5.0,
    ):
        self._max_tilt = float(max_tilt)
        self._yaw_rate_gain = float(yaw_rate_gain)
        self._z_gain = float(z_gain)

        try:
            import hid  # type: ignore
        except Exception:
            self._hid = None
        else:
            try:
                self._hid = hid.device()
                self._hid.open(self.VENDOR_ID, self.PRODUCT_ID)
                self._hid.set_nonblocking(True)
            except Exception:
                self._hid = None

    def read(self, dt: float) -> PilotCommand:
        if self._hid is None:
            return PilotCommand()
        try:
            data = self._hid.read(64)
        except OSError:
            data = None
        if not data or len(data) < 9 or data[0] != 0x01:
            return PilotCommand()

        lx, ly, rx, ry = data[1], data[2], data[3], data[4]
        norm_lx = (lx - 127) / 127.0
        norm_ly = (127 - ly) / 127.0
        norm_rx = (rx - 127) / 127.0
        norm_ry = (127 - ry) / 127.0

        z_delta = norm_ly * self._z_gain * dt
        roll_t = max(-self._max_tilt, min(self._max_tilt, norm_rx * self._max_tilt))
        pitch_t = max(-self._max_tilt, min(self._max_tilt, norm_ry * self._max_tilt))
        yaw_rate = -norm_lx * self._yaw_rate_gain
        return PilotCommand(roll_target=roll_t, pitch_target=pitch_t, yaw_rate=yaw_rate, z_setpoint_delta=z_delta)

    def close(self) -> None:
        if getattr(self, "_hid", None) is not None:
            try:
                self._hid.close()
            except Exception:
                pass

