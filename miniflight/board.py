"""
Board interface defining the abstraction between firmware and hardware targets.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Sequence, Tuple

from common.types import SensorReadings


class Board(ABC):
    """
    Abstract interface that every hardware target must implement.
    The board is responsible for wiring sensors, estimators, and actuators.
    Firmware talks only to this interface.
    """

    @abstractmethod
    def read_sensors(self) -> SensorReadings:
        """Return the latest raw sensor readings."""

    @abstractmethod
    def write_actuators(self, commands: Sequence[float]) -> None:
        """Send actuator commands produced by the controller."""

    def motor_geometry(self) -> Optional[Tuple[Sequence, Sequence]]:
        """
        Optional hook to describe motor layout (positions, spins) so generic mixers
        can be constructed at the firmware level.
        """
        return None

    def close(self) -> None:
        """Optional cleanup hook."""
        return None


