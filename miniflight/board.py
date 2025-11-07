"""
Board interface defining the abstraction between firmware and hardware targets.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Sequence, Tuple

from common.types import StateEstimate


class Board(ABC):
    """
    Abstract interface that every hardware target must implement.
    The board is responsible for wiring HAL drivers, estimators, and actuators.
    Firmware talks only to this interface.
    """

    def __init__(self, controller):
        self.controller = controller

    @abstractmethod
    def read_state(self) -> Tuple[StateEstimate, float]:
        """Return the latest state estimate and timestamp (seconds)."""

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


