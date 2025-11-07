#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, setup HAL and scheduler, and run control loop.
"""
import os

from common.logger import get_logger
from common.scheduler import Scheduler
from miniflight.control import StabilityController, GenericMixer
from miniflight.board   import Board
from miniflight.estimate import MahonyEstimator

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
        self.estimator = MahonyEstimator()

        # Board
        self.board = init_board(target, self.controller, self.dt)

        # Mixer inferred from board motor geometry if provided
        geometry = self.board.motor_geometry()
        self.mixer = None
        if geometry is not None:
            positions, spins = geometry
            self.mixer = GenericMixer(positions, spins)
        logger.info(f"Board initialized ({type(self.board).__name__})")


    def run(self):
        def step():
            readings = self.board.read_sensors()
            state = self.estimator.update(readings, self.dt)
            cmd = self.controller.update(state, self.dt)
            motors = list(self.mixer.mix(cmd)) if self.mixer is not None else cmd
            self.board.write_actuators(motors)

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