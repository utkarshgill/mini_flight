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
class PilotCommand:
    """
    High-level pilot input used by firmware:
    - roll_target, pitch_target: desired angles (rad) for self-leveling (angle mode)
    - yaw_rate: desired yaw rate (rad/s)
    - z_setpoint_delta: change to altitude setpoint (m) to apply this tick
    """
    roll_target: float = 0.0
    pitch_target: float = 0.0
    yaw_rate: float = 0.0
    z_setpoint_delta: float = 0.0


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


