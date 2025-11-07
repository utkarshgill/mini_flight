"""
Interface definitions for sensors, actuators, controllers, and estimators.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from common.types import ImuSample, StateEstimate


class Sensor(ABC):
    """Abstract base for sensor implementations."""

    def read(self) -> Any:
        """Read data from the sensor."""
        raise NotImplementedError


class Actuator(ABC):
    """Abstract base for actuator implementations."""

    def write(self, command: Any) -> None:
        """Send a command to the actuator."""
        raise NotImplementedError


class Controller(ABC):
    """Abstract base for control algorithms."""

    @abstractmethod
    def update(self, state: StateEstimate, dt: float):
        """Compute control command given a state estimate and timestep."""


class Estimator(ABC):
    """Abstract base for sensor fusion / state estimators."""

    @abstractmethod
    def update(self, sample: ImuSample, dt: float) -> StateEstimate:
        """Update state estimate from the latest sensor measurement."""