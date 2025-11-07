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

try:
    import hid
except ImportError:  # pragma: no cover - dependency optional
    hid = None


class SimWorld:
    """Encapsulates the physics world, vehicle, and input devices."""

    def __init__(self, controller, dt: float, thrust_gain=5.0, max_tilt=0.5, yaw_rate_gain=np.pi / 2):
        self.controller = controller
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

        # Input configuration
        self._thrust_gain = thrust_gain
        self._max_tilt = max_tilt
        self._yaw_rate_gain = yaw_rate_gain
        self._hid = None
        if hid:
            try:
                self._hid = hid.device()
                self._hid.open(0x054C, 0x0CE6)  # DualSense VID/PID
                self._hid.set_nonblocking(True)
            except Exception:
                self._hid = None

    # -- Firmware-facing helpers -------------------------------------------------

    def motor_geometry(self):
        return self._motor_positions, self._motor_spins

    def update_pilot_inputs(self):
        """Poll input devices and update controller setpoints."""
        keyboard_state = self._poll_renderer_input()
        handled = self._update_from_dualsense()
        if not handled:
            self._update_from_keyboard(keyboard_state)

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

    def _update_from_dualsense(self) -> bool:
        if self._hid is None or self.controller is None:
            return False
        try:
            data = self._hid.read(64)
        except OSError:
            data = None
        if not data or len(data) < 9 or data[0] != 0x01:
            return False

        def deadzone(val, thresh=0.1):
            return val if abs(val) > thresh else 0.0

        lx, ly, rx, ry = data[1], data[2], data[3], data[4]
        norm_lx = deadzone((lx - 127) / 127.0)
        norm_ly = deadzone((127 - ly) / 127.0)
        norm_rx = deadzone((rx - 127) / 127.0)
        norm_ry = deadzone((127 - ry) / 127.0)

        dt = self.dt
        self.controller.z_setpoint += norm_ly * self._thrust_gain * dt
        self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))
        self.controller.set_attitude_target(
            np.clip(norm_rx * self._max_tilt, -self._max_tilt, self._max_tilt),
            np.clip(norm_ry * self._max_tilt, -self._max_tilt, self._max_tilt),
            wrap_angle(self.controller.yaw_setpoint - norm_lx * self._yaw_rate_gain * dt),
        )
        return True

    def _update_from_keyboard(self, keys):
        if self.controller is None:
            return
        dt = self.dt
        if keys.get("w", False):
            self.controller.z_setpoint += self._thrust_gain * dt
        if keys.get("s", False):
            self.controller.z_setpoint -= self._thrust_gain * dt
        self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))

        roll_t = (
            -self._max_tilt
            if keys.get("left", False)
            else self._max_tilt
            if keys.get("right", False)
            else 0.0
        )
        pitch_t = (
            self._max_tilt
            if keys.get("up", False)
            else -self._max_tilt
            if keys.get("down", False)
            else 0.0
        )
        yaw_sp = self.controller.yaw_setpoint
        if keys.get("a", False):
            yaw_sp += self._yaw_rate_gain * dt
        if keys.get("d", False):
            yaw_sp -= self._yaw_rate_gain * dt
        yaw_sp = wrap_angle(yaw_sp)
        self.controller.set_attitude_target(roll_t, pitch_t, yaw_sp)

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

    def __init__(self, controller, dt: float = 0.01):
        self.dt = dt
        self._world = SimWorld(controller, dt)
        self._imu = SimImuSensor(self._world)
        self._motors = SimMotorActuator(self._world)

    def read_sensors(self) -> SensorReadings:
        self._world.update_pilot_inputs()
        imu_sample = self._imu.read()
        altitude = float(self._world.quad.position.v[2])
        return SensorReadings(imu=imu_sample, timestamp=imu_sample.timestamp, altitude=altitude)

    def write_actuators(self, commands):
        self._motors.write(commands)

    def motor_geometry(self):
        return self._world.motor_geometry()

    def close(self):
        self._world.close()


__all__ = ["SimBoard"]
