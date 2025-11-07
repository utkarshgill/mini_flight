#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, setup HAL and scheduler, and run control loop.
"""
import os

from common.logger import get_logger
from common.scheduler import Scheduler
from miniflight.control import StabilityController, GenericMixer
from miniflight.board   import Board

logger = get_logger("firmware")

def init_board(target_name: str, controller: StabilityController, dt: float) -> Board:
    """Instantiate the appropriate hardware board implementation."""
    target = (target_name or "sim").lower()
    if target == "sim":
        from target.simulator import SimBoard

        return SimBoard(controller=controller, dt=dt)
    raise NotImplementedError(f"Unsupported target '{target}'")

class Controls:
    """Controls-style loop: update() -> state_control() -> publish() -> run()."""

    def __init__(self, target: str | None, dt: float = 0.01):
        self.dt = dt
        self.controller = StabilityController()

        # Board
        self.board = init_board(target, self.controller, self.dt)

        # Mixer inferred from board motor geometry if provided
        geometry = self.board.motor_geometry()
        self.mixer = None
        if geometry is not None:
            positions, spins = geometry
            self.mixer = GenericMixer(positions, spins)
        logger.info(f"Board initialized ({type(self.board).__name__})")

        # Cached IO
        self._state = None
        self._time = 0.0
        self._motors = None

    def update(self):
        # Read current state estimate and time from HAL
        self._state, self._time = self.board.read_state()

    def state_control(self):
        # Compute command from controller
        cmd = self.controller.update(self._state, self.dt)
        # Mix to motor thrusts if mixer available
        if self.mixer is not None:
            self._motors = list(self.mixer.mix(cmd))
        else:
            self._motors = cmd

    def publish(self):
        # Write motor commands to HAL
        self.board.write_actuators(self._motors)

    def run(self):
        # Single scheduled step at dt calling update -> state_control -> publish
        def step():
            self.update()
            self.state_control()
            self.publish()

        scheduler = Scheduler()
        scheduler.add_task(step, period=self.dt)
        logger.info("Starting controls loop")
        scheduler.run()


def main():
    target_env = os.environ.get("TARGET")
    target = target_env.lower() if target_env else None
    Controls(target, dt=0.01).run()

if __name__ == "__main__":
    main() 