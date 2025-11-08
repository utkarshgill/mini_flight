"""
Simulated hardware board: glues firmware-facing board interface to the physics engine.
"""

from __future__ import annotations

import numpy as np

from common.math import wrap_angle, GRAVITY
from common.types import ImuSample, SensorReadings, StateEstimate
from miniflight.board import Board
from common.interface import Sensor, Actuator
from sim import (
    Vector3D,
    Quaternion,
    World,
    RungeKuttaIntegrator,
    GravitationalForce,
    GroundCollision,
    Motor,
    Renderer,
)
from sim.engine import Quadcopter


class SimWorld:
    """Encapsulates the physics world, vehicle, and input devices."""

    def __init__(self, dt: float, thrust_gain=5.0, max_tilt=0.5, yaw_rate_gain=np.pi / 2):
        self.dt = dt

        # Build physics world and quadrotor
        self.quad = Quadcopter(position=Vector3D(0, 0, 0.1))
        self.quad.integrator = RungeKuttaIntegrator()
        self.quad.urdf_filename = "quadrotor.urdf"

        L = self.quad.arm_length
        diag = L / np.sqrt(2)
        motor_positions = [
            Vector3D(diag, diag, 0),
            Vector3D(-diag, diag, 0),
            Vector3D(-diag, -diag, 0),
            Vector3D(diag, -diag, 0),
        ]
        spins = [1, -1, 1, -1]
        for idx, (r, s) in enumerate(zip(motor_positions, spins)):
            self.quad.add_actuator(
                Motor(
                    idx,
                    r_body=r,
                    spin=s,
                    thrust_noise_std=0.0,
                    torque_noise_std=0.0,
                )
            )
        self._motor_positions = motor_positions
        self._motor_spins = spins

        self.world = World(
            forces=[GravitationalForce(), GroundCollision(ground_level=0.0, restitution=0.5)],
            dt=self.dt,
        )
        self.world.add_body(self.quad)
        self.world.update()  # populate initial state

        self._spawn_pose = {
            "position": tuple(self.quad.position.v.tolist()),
            "velocity": tuple(self.quad.velocity.v.tolist()),
            "acceleration": tuple(self.quad.acceleration.v.tolist()),
            "orientation": tuple(self.quad.orientation.q.tolist()),
            "angular_velocity": tuple(self.quad.angular_velocity.v.tolist()),
        }
        self._space_held = False
        self._latest_input = {}

        # Renderer
        try:
            self.renderer = Renderer(self.world, config="X", gui=True)
        except Exception as exc:  # pragma: no cover - renderer optional
            print(f"Renderer init error: {exc}")
            self.renderer = None

        # Input configuration (used by renderer for keyboard capture)
        self._thrust_gain = thrust_gain
        self._max_tilt = max_tilt
        self._yaw_rate_gain = yaw_rate_gain
        # No direct HID/gamepad handling in the simulator layer

    # -- Firmware-facing helpers -------------------------------------------------

    def motor_geometry(self):
        return self._motor_positions, self._motor_spins

    def get_input_state(self):
        """Return the latest input keys from the renderer (non-blocking)."""
        return self._poll_renderer_input()

    def imu_sample(self) -> ImuSample:
        """Return synthetic IMU sample: specific force and body rates."""
        a_world = self.quad.acceleration.v
        g_world = np.array([0.0, 0.0, -GRAVITY])
        f_world = a_world - g_world
        f_body_vec = self.quad.orientation.conjugate().rotate(Vector3D(*f_world))
        accel = tuple(f_body_vec.v.tolist())
        gyro = tuple(self.quad.angular_velocity.v.tolist())
        return ImuSample(accel=accel, gyro=gyro, timestamp=self.time())

    def state(self) -> StateEstimate:
        return StateEstimate(
            position=Vector3D(*self.quad.position.v.tolist()),
            velocity=Vector3D(*self.quad.velocity.v.tolist()),
            orientation=Quaternion(*self.quad.orientation.q.tolist()),
            angular_velocity=Vector3D(*self.quad.angular_velocity.v.tolist()),
        )

    def time(self) -> float:
        return self.world.time

    def apply_motor_commands(self, commands):
        """Advance physics one tick with the provided motor commands."""
        self.quad.motor_thrusts = commands
        self.world.update()

        input_state = self._latest_input
        if " " in input_state:
            space_down = bool(input_state.get(" "))
        else:
            space_down = False
        if space_down and not self._space_held:
            self._respawn_quad()
        self._space_held = space_down

        if self.renderer:
            try:
                self.renderer.draw()
            except Exception as exc:  # pragma: no cover
                print(f"Renderer draw error: {exc}")

    def close(self):
        if self.renderer:
            try:
                self.renderer.shutdown()
            except Exception:
                pass

    # -- Input handling ----------------------------------------------------------

    def _poll_renderer_input(self):
        state = {}
        if self.renderer and hasattr(self.renderer, "get_input_state"):
            try:
                state = self.renderer.get_input_state() or {}
            except Exception:
                state = {}
        self._latest_input = dict(state)
        return state

    # NOTE: Controller coupling removed; inputs are consumed by firmware via command source.

    def _respawn_quad(self):
        self.quad.position = Vector3D(*self._spawn_pose["position"])
        self.quad.velocity = Vector3D(*self._spawn_pose["velocity"])
        self.quad.acceleration = Vector3D(*self._spawn_pose["acceleration"])
        self.quad.orientation = Quaternion(*self._spawn_pose["orientation"])
        self.quad.orientation.normalize()
        self.quad.angular_velocity = Vector3D(*self._spawn_pose["angular_velocity"])
        self.quad.integrator = RungeKuttaIntegrator()
        actuator_count = (
            len(getattr(self.quad, "actuators", []))
            or len(getattr(self.quad, "motor_thrusts", []))
            or 4
        )
        self.quad.motor_thrusts = [0.0] * actuator_count
        self.world.current_state, self.world.current_flat = self.world.get_state()


class SimImuSensor(Sensor):
    """Virtual IMU driver for the simulator board."""

    def __init__(self, world: SimWorld):
        self._world = world

    def read(self) -> ImuSample:
        return self._world.imu_sample()


class SimMotorActuator(Actuator):
    """Virtual motor driver that feeds thrust commands into the simulator."""

    def __init__(self, world: SimWorld):
        self._world = world

    def write(self, command) -> None:
        self._world.apply_motor_commands(command)


class SimBoard(Board):
    """Board implementation backed by the simulator world."""

    def __init__(self, dt: float = 0.01):
        self.dt = dt
        self._world = SimWorld(dt)
        self._imu = SimImuSensor(self._world)
        self._motors = SimMotorActuator(self._world)

    def read_sensors(self) -> SensorReadings:
        imu_sample = self._imu.read()
        altitude = float(self._world.quad.position.v[2])
        return SensorReadings(imu=imu_sample, timestamp=imu_sample.timestamp, altitude=altitude)

    def write_actuators(self, commands):
        self._motors.write(commands)

    def motor_geometry(self):
        return self._world.motor_geometry()

    def close(self):
        self._world.close()

    # Expose world for constructing command sources in target init (firmware layer)
    @property
    def world(self) -> SimWorld:
        return self._world


__all__ = ["SimBoard"]
