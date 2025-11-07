"""
Shared data structures for firmware ↔ HAL ↔ simulator boundaries.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from common.math import Vector3D, Quaternion


@dataclass(frozen=True)
class ImuSample:
    """Raw IMU sample consisting of acceleration and angular rates."""

    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    timestamp: float


@dataclass(frozen=True)
class StateEstimate:
    """Estimated vehicle state used by the controller."""

    position: Vector3D
    velocity: Vector3D
    orientation: Quaternion
    angular_velocity: Vector3D


@dataclass(frozen=True)
class SensorReadings:
    """Collection of raw sensor readings used by the estimator."""

    imu: ImuSample
    timestamp: float
    altitude: Optional[float] = None


