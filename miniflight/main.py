#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, setup HAL and scheduler, and run control loop.
"""
import os
import numpy as np

from common.logger import get_logger
from common.scheduler import Scheduler
from miniflight.control import StabilityController, GenericMixer
from miniflight.board   import Board
from miniflight.estimate import MahonyEstimator
from common.math import wrap_angle
from miniflight.input import KeyboardCommandSource, DualSenseCommandSource
from common.interface import CommandSource

logger = get_logger("firmware")

def init_board(target_name: str, dt: float) -> tuple[Board, CommandSource | None]:
    """Instantiate the board and an optional command source for pilot input."""
    target = (target_name or "sim").lower()
    if target == "sim":
        from target.simulator import SimBoard

        board = SimBoard(dt=dt)
        # Choose input source
        input_kind = (os.environ.get("INPUT") or "keyboard").lower()
        cmd_src: CommandSource | None = None
        if input_kind == "dualsense":
            cmd_src = DualSenseCommandSource()
        else:
            # keyboard via renderer embedded in sim world
            get_keys = lambda: board.world.get_input_state()
            cmd_src = KeyboardCommandSource(get_keys=get_keys)
        return board, cmd_src
    raise NotImplementedError(f"Unsupported target '{target}'")

class Controls:
    """Controls-style loop: update() -> state_control() -> publish() -> run()."""

    def __init__(self, target: str | None, dt: float = 0.01):
        self.dt = dt
        self.controller = StabilityController()
        self.estimator = MahonyEstimator()

        # Board
        self.board, self.command_source = init_board(target, self.dt)

        # Mixer inferred from board motor geometry if provided
        geometry = self.board.motor_geometry()
        self.mixer = None
        if geometry is not None:
            positions, spins = geometry
            self.mixer = GenericMixer(positions, spins)
        logger.info(f"Board initialized ({type(self.board).__name__})")


    def run(self):
        def step():
            # Read pilot command and update controller setpoints
            if self.command_source is not None:
                pc = self.command_source.read(self.dt)
                # Altitude target integrates pilot delta
                self.controller.z_setpoint += pc.z_setpoint_delta
                self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))
                # Yaw integrates rate command
                self.controller.yaw_setpoint = wrap_angle(self.controller.yaw_setpoint + pc.yaw_rate * self.dt)
                self.controller.set_attitude_target(pc.roll_target, pc.pitch_target, self.controller.yaw_setpoint)

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