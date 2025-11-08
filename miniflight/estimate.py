"""State estimation primitives shared across firmware, sim, and configurator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from common.interface import Estimator
from common.math import Quaternion, Vector3D
from common.types import SensorReadings, StateEstimate


@dataclass
class MahonyGains:
    kp: float = 0.5
    ki: float = 0.001  # Very small for slow gyro bias correction


class MahonyEstimator(Estimator):
    """Mahony AHRS: attitude estimation from gyro + accelerometer."""

    def __init__(self, gains: MahonyGains | None = None):
        gains = gains or MahonyGains()
        self.kp = gains.kp
        self.ki = gains.ki
        self._bias = Vector3D()
        self._q = Quaternion()  # world (z-up) → body
        self._initialized = False
        self._last_state: Optional[StateEstimate] = None
        self._last_alt: Optional[float] = None
        self._init_samples = []
        self._init_sample_count = 20  # Average 20 samples before initializing

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def initialization_progress(self) -> float:
        if self._initialized:
            return 1.0
        if not self._init_samples:
            return 0.0
        return min(len(self._init_samples) / self._init_sample_count, 1.0)

    def set_initial_bias_deg(self, gx_deg: float, gy_deg: float, gz_deg: float) -> None:
        """Seed gyro bias (in deg/s). Helpful right after bias calibration."""
        self._bias = Vector3D(
            math.radians(gx_deg),
            math.radians(gy_deg),
            math.radians(gz_deg),
        )

    def reset(self) -> None:
        self._bias = Vector3D()
        self._q = Quaternion()
        self._initialized = False
        self._last_state = None
        self._last_alt = None
        self._init_samples = []

    def update(self, readings: SensorReadings, dt: float) -> StateEstimate:
        if dt <= 0.0:
            return self._fallback()

        sample = readings.imu
        ax, ay, az = sample.accel  # specific force (points up at rest)
        # Input gyro is in degrees/sec; convert to rad/sec for integration
        gx, gy, gz = [math.radians(v) for v in sample.gyro]

        # Convert specific force to gravity direction (pointing down)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return self._fallback()

        # Measured gravity direction (opposite of specific force)
        grav_x = -ax / norm
        grav_y = -ay / norm
        grav_z = -az / norm

        if not self._initialized:
            # Collect samples for stable initialization
            self._init_samples.append((grav_x, grav_y, grav_z))
            if len(self._init_samples) < self._init_sample_count:
                return self._fallback()
            
            # Average gravity samples to reduce noise
            avg_gx = sum(s[0] for s in self._init_samples) / len(self._init_samples)
            avg_gy = sum(s[1] for s in self._init_samples) / len(self._init_samples)
            avg_gz = sum(s[2] for s in self._init_samples) / len(self._init_samples)
            
            # Initialize attitude from averaged gravity: z-up frame, gravity points down
            roll0 = math.atan2(-avg_gy, -avg_gz)
            pitch0 = math.atan2(avg_gx, math.sqrt(avg_gy * avg_gy + avg_gz * avg_gz))
            self._q = Quaternion.from_euler(roll0, pitch0, 0.0)
            self._q.normalize()
            self._initialized = True
            self._init_samples = []  # Clear samples after initialization

        # Predicted gravity in body frame
        v = self._q.conjugate().rotate(Vector3D(0, 0, -1))
        vx, vy, vz = v.v

        # Skip accel correction if magnitude deviates too far from 1g (tight threshold)
        if abs(norm - 1.0) > 0.05:
            ex = ey = ez = 0.0
        else:
            # Error between measured and predicted gravity
            ex = grav_y * vz - grav_z * vy
            ey = grav_z * vx - grav_x * vz
            ez = grav_x * vy - grav_y * vx

        # For IMU-only (no magnetometer), do not correct yaw from accelerometer
        ez = 0.0

        # Only adapt bias when nearly still (accel trustworthy and angular rate low)
        ang_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        if self.ki > 0.0 and abs(norm - 1.0) <= 0.02 and ang_mag < math.radians(20.0):
            # Integrate bias for roll/pitch only; ignore yaw without mag
            self._bias += Vector3D(ex, ey, 0.0) * (self.ki * dt)

        # Corrected gyro
        gx_c = gx + self.kp * ex + self._bias.v[0]
        gy_c = gy + self.kp * ey + self._bias.v[1]
        gz_c = gz + self._bias.v[2]  # no accel-based yaw correction

        # Integrate quaternion: q̇ = 0.5 * q ⊗ ω
        omega = Quaternion(0.0, gx_c, gy_c, gz_c)
        q_dot = self._q * omega * 0.5
        self._q = Quaternion(*(self._q.q + q_dot.q * dt))
        self._q.normalize()

        # Position / velocity placeholders (can be extended with baro/GNSS later)
        position = self._last_state.position if self._last_state else Vector3D()
        velocity = self._last_state.velocity if self._last_state else Vector3D()
        angular_velocity = Vector3D(*sample.gyro)

        if readings.altitude is not None:
            z = float(readings.altitude)
            if self._last_alt is not None and dt > 0.0:
                vz = (z - self._last_alt) / dt
            else:
                vz = 0.0
            self._last_alt = z
            position = Vector3D(0.0, 0.0, z)
            velocity = Vector3D(0.0, 0.0, vz)

        state = StateEstimate(
            position=position,
            velocity=velocity,
            orientation=self._q,
            angular_velocity=angular_velocity,
        )
        self._last_state = state
        return state

    def _fallback(self) -> StateEstimate:
        if self._last_state is not None:
            return self._last_state
        zero = Vector3D()
        quat = Quaternion()
        state = StateEstimate(position=zero, velocity=zero, orientation=quat, angular_velocity=zero)
        self._last_state = state
        self._last_alt = None
        return state


__all__ = ["MahonyEstimator", "MahonyGains"]


