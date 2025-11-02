"""Sensor fusion filters used across mini_flight components."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Dict, Optional, Sequence

from common.math import Quaternion

__all__ = ["Filter", "ComplementaryFilter"]


class Filter(ABC):
    """Abstract base class for IMU filter implementations."""

    name: str = "filter"

    @abstractmethod
    def update(
        self,
        accel: Sequence[float],
        gyro: Sequence[float],
        dt: float,
    ) -> Optional[Dict[str, object]]:
        """Advance the filter with the latest sensor sample.

        Args:
            accel: Sequence of length 3 with accelerometer readings in g.
            gyro: Sequence of length 3 with gyroscope readings in deg/s.
            dt: Time delta in seconds since the previous update.

        Returns:
            Optional mapping with fused orientation data. Returning ``None``
            indicates that no update is available (e.g. insufficient data).
        """


class ComplementaryFilter(Filter):
    """Gyro-integrating complementary filter with accelerometer correction."""

    name = "complementary"

    def __init__(self, *, alpha: float = 0.98) -> None:
        self.alpha = float(alpha)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._initialized = False

    def reset(self) -> None:
        """Reset the internal state to zero attitude."""

        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._initialized = False

    def update(
        self,
        accel: Sequence[float],
        gyro: Sequence[float],
        dt: float,
    ) -> Optional[Dict[str, object]]:
        if dt <= 0.0 or not accel or not gyro:
            return None

        try:
            ax, ay, az = (float(accel[0]), float(accel[1]), float(accel[2]))
            gx, gy, gz = (float(gyro[0]), float(gyro[1]), float(gyro[2]))
        except (IndexError, ValueError, TypeError):
            return None

        self._integrate_gyro(gx, gy, gz, dt)
        self._apply_accel_correction(ax, ay, az)

        quaternion = Quaternion.from_euler(self._roll, self._pitch, self._yaw)
        quaternion.normalize()

        return {
            "filter": self.name,
            "quaternion": [float(v) for v in quaternion.q.tolist()],
            "euler_deg": [
                math.degrees(self._roll),
                math.degrees(self._pitch),
                math.degrees(self._yaw),
            ],
            "euler_rad": [self._roll, self._pitch, self._yaw],
        }

    def _integrate_gyro(self, gx: float, gy: float, gz: float, dt: float) -> None:
        # Convert from deg/s to rad/s and integrate
        self._roll += math.radians(gx) * dt
        self._pitch += math.radians(gy) * dt
        self._yaw += math.radians(gz) * dt
        self._yaw = _wrap_angle(self._yaw)

    def _apply_accel_correction(self, ax: float, ay: float, az: float) -> None:
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-3:
            return

        ax /= norm
        ay /= norm
        az /= norm

        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if not self._initialized:
            self._roll = roll_acc
            self._pitch = pitch_acc
            self._yaw = 0.0
            self._initialized = True
            return

        self._roll = self.alpha * self._roll + (1.0 - self.alpha) * roll_acc
        self._pitch = self.alpha * self._pitch + (1.0 - self.alpha) * pitch_acc


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

