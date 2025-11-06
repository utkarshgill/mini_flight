import numpy as np
from miniflight.hal import HAL
from sim import Vector3D, Quaternion, World, RungeKuttaIntegrator, GravitationalForce, GroundCollision, IMUSensor, Motor, Renderer
from sim.engine import Quadcopter
from common.math import wrap_angle
try:
    import hid
except ImportError:
    hid = None

class Board(HAL):
    """Firmware-side simulated HAL driving core physics."""
    def __init__(self, dt: float = 0.01, controller=None, config=None):
        super().__init__(config)
        self.dt = dt
        self.controller = controller
        # Build physics world and quad
        self.quad = Quadcopter(position=Vector3D(0, 0, 0.1))
        self.quad.integrator = RungeKuttaIntegrator()
        self.quad.add_sensor(IMUSensor(accel_noise_std=0.0, gyro_noise_std=0.0))
        # Inform renderer which URDF to load for the quad
        self.quad.urdf_filename = "quadrotor.urdf"
        # X-configuration motors positions and spins
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
            self.quad.add_actuator(Motor(idx, r_body=r, spin=s,
                                          thrust_noise_std=0.0, torque_noise_std=0.0))
        # Create world with gravity and ground collision
        self.world = World(
            forces=[GravitationalForce(), GroundCollision(ground_level=0.0, restitution=0.5)],
            dt=self.dt
        )
        self.world.add_body(self.quad)
        # Initial update to populate state
        self.world.update()
        self._spawn_pose = {
            "position": tuple(self.quad.position.v.tolist()),
            "velocity": tuple(self.quad.velocity.v.tolist()),
            "acceleration": tuple(self.quad.acceleration.v.tolist()),
            "orientation": tuple(self.quad.orientation.q.tolist()),
            "angular_velocity": tuple(self.quad.angular_velocity.v.tolist()),
        }
        self._space_held = False
        # Set up visualization
        try:
            self.renderer = Renderer(self.world, config='X', gui=True)
        except Exception as e:
            print(f"Renderer init error: {e}")

        # Input gains and HID setup (prefer DualSense if available)
        self._thrust_gain = float(config.get('thrust_gain', 5.0)) if isinstance(config, dict) else 5.0
        self._max_tilt = float(config.get('max_tilt', 0.5)) if isinstance(config, dict) else 0.5
        self._yaw_rate_gain = float(config.get('yaw_rate_gain', np.pi/2)) if isinstance(config, dict) else (np.pi/2)
        self._hid = None
        if hid:
            try:
                self._hid = hid.device()
                self._hid.open(0x054C, 0x0CE6)  # DualSense VID/PID
                self._hid.set_nonblocking(True)
            except Exception:
                self._hid = None

    def read(self):
        """Return the primary quad Body and the current simulation time.
        Also updates controller setpoints from input devices before control."""
        # Update controller setpoints from input (DualSense preferred)
        if self.controller is not None:
            self._update_input_setpoints()
        # Return the Body instance and current sim time
        return self.quad, self.world.time

    def write(self, commands):
        """Apply motor commands, handle keyboard input, advance physics, and render."""
        # Update motor thrusts
        self.quad.motor_thrusts = commands
        # Advance simulation
        self.world.update()
        input_state = {}
        if hasattr(self, 'renderer'):
            if hasattr(self.renderer, 'get_input_state'):
                try:
                    input_state = self.renderer.get_input_state()
                except Exception as e:
                    print(f"Renderer input error: {e}")
            try:
                self.renderer.draw()
            except Exception as e:
                print(f"Renderer draw error: {e}")
        space_down = bool(input_state.get(' '))
        if space_down and not self._space_held:
            self._respawn_quad()
        self._space_held = space_down
        if space_down:
            input_state.pop(' ', None)
        return commands

    # --- Input helpers ---
    def _update_input_setpoints(self):
        dt = self.dt
        # Prefer DualSense HID if available and readable
        if self._hid is not None:
            try:
                data = self._hid.read(64)
            except OSError:
                data = None
            if data and len(data) >= 9 and data[0] == 0x01:
                # Cross button and sticks
                lx, ly, rx, ry = data[1], data[2], data[3], data[4]
                def deadzone(v, dz=0.1):
                    return v if abs(v) > dz else 0.0
                norm_lx = deadzone((lx - 127)/127.0)
                norm_ly = deadzone((127 - ly)/127.0)
                norm_rx = deadzone((rx - 127)/127.0)
                norm_ry = deadzone((127 - ry)/127.0)
                # Altitude setpoint
                self.controller.z_setpoint += norm_ly * self._thrust_gain * dt
                self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))
                # Attitude setpoints
                self.controller.set_attitude_target(
                    np.clip(norm_rx * self._max_tilt, -self._max_tilt, self._max_tilt),
                    np.clip(norm_ry * self._max_tilt, -self._max_tilt, self._max_tilt),
                    wrap_angle(self.controller.yaw_setpoint - norm_lx * self._yaw_rate_gain * dt)
                )
                return
        # Fallback to keyboard input via renderer
        input_state = {}
        if hasattr(self, 'renderer') and hasattr(self.renderer, 'get_input_state'):
            try:
                input_state = self.renderer.get_input_state()
            except Exception:
                input_state = {}
        # Throttle W/S
        if input_state.get('w', False):
            self.controller.z_setpoint += self._thrust_gain * dt
        if input_state.get('s', False):
            self.controller.z_setpoint -= self._thrust_gain * dt
        self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))
        # Roll/Pitch arrows
        roll_t = (-self._max_tilt if input_state.get('left', False)
                  else self._max_tilt if input_state.get('right', False)
                  else 0.0)
        pitch_t = (self._max_tilt if input_state.get('up', False)
                   else -self._max_tilt if input_state.get('down', False)
                   else 0.0)
        # Yaw A/D
        yaw_sp = self.controller.yaw_setpoint
        if input_state.get('a', False):
            yaw_sp += self._yaw_rate_gain * dt
        if input_state.get('d', False):
            yaw_sp -= self._yaw_rate_gain * dt
        yaw_sp = wrap_angle(yaw_sp)
        self.controller.set_attitude_target(roll_t, pitch_t, yaw_sp)

    def _respawn_quad(self) -> None:
        self.quad.position = Vector3D(*self._spawn_pose["position"])
        self.quad.velocity = Vector3D(*self._spawn_pose["velocity"])
        self.quad.acceleration = Vector3D(*self._spawn_pose["acceleration"])
        self.quad.orientation = Quaternion(*self._spawn_pose["orientation"])
        self.quad.orientation.normalize()
        self.quad.angular_velocity = Vector3D(*self._spawn_pose["angular_velocity"])
        self.quad.integrator = RungeKuttaIntegrator()
        actuator_count = len(getattr(self.quad, 'actuators', [])) or len(getattr(self.quad, 'motor_thrusts', [])) or 4
        self.quad.motor_thrusts = [0.0] * actuator_count
        self.world.current_state, self.world.current_flat = self.world.get_state()

__all__ = ["Board"]