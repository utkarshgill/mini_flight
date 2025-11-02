#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, setup HAL and scheduler, and run control loop.
"""
import json
import os

from common.logger import get_logger
from common.scheduler import Scheduler
from miniflight.control import StabilityController, GenericMixer
from miniflight.hal      import HAL
from miniflight.hil      import Keyboard, DualSense

logger = get_logger("firmware")

def load_config(path: str = "config.json") -> dict:
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        logger.info(f"Config file '{path}' not found, using defaults.")
        return {}

def init_hal(config: dict, controller: StabilityController, dt: float):
    """Initialize the appropriate HAL based on target: use Board for sim, generic HAL otherwise."""
    target_name = config.get("target")
    if target_name == "sim":
        from target.simulator import Board
        return Board(dt=dt, controller=controller, config=config)
    # Default to generic HAL
    return HAL(config)

def main():
    # Load configuration
    config = load_config()
    # Override target via environment variable
    target_env = os.environ.get("TARGET")
    if target_env:
        config["target"] = target_env.lower()
    logger.info("Configuration loaded")

    # Time step for control and simulation
    dt = config.get("dt", 0.01)

    # Instantiate controller and HAL
    controller = StabilityController()
    hal = init_hal(config, controller, dt)
    # Attach HIL devices in main
    hal.keyboard = Keyboard()
    hal.dualsense = DualSense()
    hal.hil = hal.dualsense if getattr(hal.dualsense, 'h', None) else hal.keyboard
    # Attach mixer based on HAL actuators
    positions = [act.r_body for act in hal.quad.actuators]
    spins = [act.spin for act in hal.quad.actuators]
    hal.mixer = GenericMixer(positions, spins)
    logger.info(f"HAL initialized ({type(hal).__name__})")

    # Define step callback with explicit pipeline
    def step():
        # 1) Read current body state and time
        body, t = hal.read()
        logger.info(f"Step @ t={t:.3f}, pos=({body.position.v[0]:.2f},{body.position.v[1]:.2f},{body.position.v[2]:.2f})")
        # 2) Human-in-the-loop update
        hal.hil.update(controller, dt)
        # 3) Control based on current body
        cmd = controller.update(body, dt)
        # 4) Mix to per-motor thrusts if available
        motors = list(hal.mixer.mix(cmd)) if hasattr(hal, 'mixer') else cmd
        # 5) Write motor commands
        hal.write(motors)

    # Setup and run scheduler with explicit pipeline
    scheduler = Scheduler()
    scheduler.add_task(step, period=dt)
    logger.info("Starting scheduler loop")
    scheduler.run()

if __name__ == "__main__":
    main() 