"""
Hardware Abstraction Layer (HAL)
Provides a unified interface for sensors, actuators, and controllers.
"""

class HAL:
    """
    Hardware Abstraction Layer (HAL) base class.
    Provides abstract read/write interfaces to communicate with a controller.
    """
    def __init__(self):
        """Initialize HAL base class."""

    def read(self):
        """Return the latest state estimate (and optionally timestamp). Must be implemented by subclass."""
        raise NotImplementedError("HAL.read() must be implemented by subclass")

    def write(self, commands):
        """Write actuator commands. Must be implemented by subclass."""
        raise NotImplementedError("HAL.write() must be implemented by subclass") 