import numpy as np
from miniflight.hal import HAL
from sim import Vector3D, Quaternion, World, RungeKuttaIntegrator, GravitationalForce, GroundCollision, IMUSensor, Motor, Renderer
from sim.engine import Quadcopter

class Board(HAL):
    """Firmware-side simulated HAL driving core physics."""
    def __init__(self, dt: float = 0.01, controller=None, config=None):
        super().__init__(config)
        self.dt = dt
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

    def read(self):
        """Return the primary quad Body and the current simulation time."""
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
        if hasattr(self, 'hil') and hasattr(self.hil, 'update_key_state'):
            try:
                self.hil.update_key_state(input_state)
            except Exception as e:
                print(f"HIL update_key_state error: {e}")
        return commands

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