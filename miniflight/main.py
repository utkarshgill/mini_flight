#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, setup HAL and scheduler, and run control loop.
"""
import os

from common.logger import get_logger
from common.scheduler import Scheduler
from miniflight.control import StabilityController, GenericMixer
from miniflight.hal      import HAL
from miniflight.hil      import Keyboard, DualSense
from miniflight.utils    import load_config

logger = get_logger("firmware")

def init_hal(config: dict, controller: StabilityController, dt: float):
    """Initialize the appropriate HAL based on target: use Board for sim, generic HAL otherwise."""
    target_name = config.get("target")
    if target_name == "sim":
        from target.simulator import Board
        return Board(dt=dt, controller=controller, config=config)
    # Default to generic HAL
    return HAL(config)

class Controls:
    """
    Controls-style loop: update() -> state_control() -> publish() -> run().
    """

    def __init__(self, config: dict):
        # dt and controller
        self.dt = config.get("dt", 0.01)
        self.controller = StabilityController()

        # HAL
        self.hal = init_hal(config, self.controller, self.dt)

        # HIL (keyboard/dualsense)
        self.hal.keyboard = Keyboard()
        self.hal.dualsense = DualSense()
        self.hal.hil = self.hal.dualsense if getattr(self.hal.dualsense, 'h', None) else self.hal.keyboard

        # Mixer inferred from actuators on the quad (sim target)
        if hasattr(self.hal, 'quad') and hasattr(self.hal.quad, 'actuators'):
            positions = [act.r_body for act in self.hal.quad.actuators]
            spins = [act.spin for act in self.hal.quad.actuators]
            self.hal.mixer = GenericMixer(positions, spins)
        logger.info(f"HAL initialized ({type(self.hal).__name__})")

        # Cached IO
        self._body = None
        self._time = 0.0
        self._motors = None

    def update(self):
        # Read current body and time from HAL
        self._body, self._time = self.hal.read()
        # Update HIL setpoints
        if hasattr(self.hal, 'hil'):
            self.hal.hil.update(self.controller, self.dt)

    def state_control(self):
        # Compute command from controller
        cmd = self.controller.update(self._body, self.dt)
        # Mix to motor thrusts if mixer available
        if hasattr(self.hal, 'mixer'):
            self._motors = list(self.hal.mixer.mix(cmd))
        else:
            self._motors = cmd

    def publish(self):
        # Write motor commands to HAL
        self.hal.write(self._motors)

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
    # Load configuration
    config = load_config()
    # Override target via environment variable
    target_env = os.environ.get("TARGET")
    if target_env:
        config["target"] = target_env.lower()
    logger.info("Configuration loaded")

    Controls(config).run()

if __name__ == "__main__":
    main() 