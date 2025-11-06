# engine.py
# Core simulation engine: messaging, scheduling, world, and high-level simulator

import numpy as np
from common.math import Vector3D, Quaternion, GRAVITY
from common.interface import Sensor, Actuator

# --- World ---
class World:
    """Simulation world orchestrating sense→control→actuate→integrate."""

    def __init__(self, forces=None, dt: float = 0.01):
        self.bodies = []
        self.time: float = 0.0
        self.dt: float = dt
        # store current state here (dict, flat array)
        self.current_state = None
        self.current_flat = None
        # list of force models to apply in actuate phase
        self.forces = forces if forces is not None else []

    def add_body(self, body):
        self.bodies.append(body)

    def _sense(self, dt: float):
        for body in self.bodies:
            for sensor in getattr(body, 'sensors', []):
                sensor.read([body], dt)
            body.acceleration = Vector3D()  # Reset acceleration after sensors read

    def _control(self, dt: float):
        for body in self.bodies:
            for controller in getattr(body, 'controllers', []):
                cmd = controller.update(body, dt)
                if cmd is not None:
                    body.control_command = cmd

    def _actuate(self, dt: float):
        for body in self.bodies:
            # apply all registered force models
            for force in self.forces:
                force.apply_to(body)
            for actuator in getattr(body, 'actuators', []):
                actuator.apply_to(body, dt)

    def _integrate(self, dt: float):
        for body in self.bodies:
            body.update(dt)

    @property
    def state_spec(self):
        """Specification of the state vector: shapes and dtypes for each component."""
        return {
            'time': {'shape': (), 'dtype': float},
            'position': {'shape': (3,), 'dtype': float},
            'velocity': {'shape': (3,), 'dtype': float},
            'orientation': {'shape': (4,), 'dtype': float},
            'angular_velocity': {'shape': (3,), 'dtype': float},
        }

    def get_state(self):
        """Return current state as a dict and as a flat numpy array."""
        # assume first body is the primary one
        body = self.bodies[0]
        s = {
            'time': self.time,
            'position': body.position.v.copy(),
            'velocity': body.velocity.v.copy(),
            'orientation': body.orientation.q.copy(),
            'angular_velocity': body.angular_velocity.v.copy(),
        }
        # flatten to [time, pos3, vel3, orient4, angvel3]
        flat = np.concatenate([
            [s['time']],
            s['position'],
            s['velocity'],
            s['orientation'],
            s['angular_velocity'],
        ]).astype(np.float32)
        return s, flat

    def run(self, render_fn=None, render_fps=50):
        """
        Run simulation and optional rendering in real time using Scheduler.
        render_fn: optional function to call each render period.
        """
        from common.scheduler import Scheduler
        sched = Scheduler()
        # Break down update into individual phases at dt intervals
        # 1) Actuate based on last control_command
        sched.add_task(lambda: self._actuate(self.dt), period=self.dt)
        # 2) Integrate dynamics and record state
        def _integrate_and_record():
            self._integrate(self.dt)
            self.time += self.dt
            self.current_state, self.current_flat = self.get_state()
        sched.add_task(_integrate_and_record, period=self.dt)
        # 3) Sense for next control cycle
        sched.add_task(lambda: self._sense(self.dt), period=self.dt)
        # Optional render task
        if render_fn:
            sched.add_task(render_fn, period=1.0/render_fps)
        sched.run()

    def update(self):
        """Advance the simulation by one internal time‐step (dt). Useful for step-by-step
        control loops where an external agent provides actions each iteration."""
        # Actuate → Integrate → Sense (control is external)
        self._actuate(self.dt)
        self._integrate(self.dt)
        self._sense(self.dt)
        # Update time and cached state vectors
        self.time += self.dt
        self.current_state, self.current_flat = self.get_state()

# --- Rigid-body dynamics relocated from dynamics.py ---

class EulerIntegrator:
    """Simple Euler integrator for Body state updates."""
    def __init__(self, angular_damp=0.98, linear_drag=0.995):
        self.angular_damp = angular_damp
        self.linear_drag = linear_drag

    def step(self, obj, dt):
        # Attitude integration: q̇ = ½ q ⊗ Ω
        omega_quat = Quaternion(0, *obj.angular_velocity.v)
        orientation_derivative = obj.orientation * omega_quat
        obj.orientation += orientation_derivative * (0.5 * dt)
        obj.orientation.normalize()

        # Angular damping
        obj.angular_velocity *= self.angular_damp

        # Linear integration
        obj.velocity += obj.acceleration * dt
        obj.velocity *= self.linear_drag
        obj.position += obj.velocity * dt

class RungeKuttaIntegrator:
    """Fourth-order Runge-Kutta integrator for Body state updates."""
    def __init__(self, angular_damp: float = 0.98, linear_drag: float = 0.995):
        self.angular_damp = angular_damp
        self.linear_drag = linear_drag

    def _rk4_step(self, func, y0: np.ndarray, t0: float, dt: float) -> np.ndarray:
        """
        Perform a single 4th-order Runge-Kutta step for ODE y' = func(t, y).
        """
        k1 = func(t0, y0)
        k2 = func(t0 + dt * 0.5, y0 + dt * k1 * 0.5)
        k3 = func(t0 + dt * 0.5, y0 + dt * k2 * 0.5)
        k4 = func(t0 + dt,       y0 + dt * k3)
        return y0 + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    def step(self, obj, dt):
        # Attitude integration (first-order)
        omega_quat = Quaternion(0, *obj.angular_velocity.v)
        q_dot = obj.orientation * omega_quat
        obj.orientation += q_dot * (0.5 * dt)
        obj.orientation.normalize()
        obj.angular_velocity *= self.angular_damp

        # Translational integration via private RK4
        state0 = np.concatenate([obj.position.v, obj.velocity.v])
        def deriv(t, state):
            vel = state[3:]
            acc = obj.acceleration.v
            return np.concatenate([vel, acc])

        new_state = self._rk4_step(deriv, state0, 0.0, dt)
        new_pos = new_state[:3]
        new_vel = new_state[3:] * self.linear_drag

        obj.position = Vector3D(*new_pos)
        obj.velocity = Vector3D(*new_vel)

class Body:
    """Rigid body with 6-DOF state."""
    def __init__(
        self,
        position=None,
        velocity=None,
        acceleration=None,
        mass=1.0,
        orientation=None,
        angular_velocity=None,
        inertia=None,
        integrator=None
    ):
        self.position = position if position is not None else Vector3D()
        self.velocity = velocity if velocity is not None else Vector3D()
        self.acceleration = acceleration if acceleration is not None else Vector3D()
        self.mass = mass
        self.orientation = orientation if orientation is not None else Quaternion()
        self.angular_velocity = angular_velocity if angular_velocity is not None else Vector3D()

        self.inertia = inertia if inertia is not None else np.eye(3)
        self.inertia_inv = np.linalg.inv(self.inertia)

        self.integrator = integrator if integrator is not None else EulerIntegrator()

        self.actuators = []
        self.sensors = []
        self.controllers = []

    def apply_force(self, force):
        self.acceleration += Vector3D(*(force.v / self.mass))

    def apply_torque(self, torque, dt=0.01):
        omega = self.angular_velocity.v
        coriolis = np.cross(omega, self.inertia.dot(omega))
        angular_accel = self.inertia_inv.dot(torque.v - coriolis)
        self.angular_velocity += Vector3D(*angular_accel) * dt

    def update(self, dt):
        self.integrator.step(self, dt)

    def add_actuator(self, actuator):
        self.actuators.append(actuator)

    def add_sensor(self, sensor):
        self.sensors.append(sensor)

    def add_controller(self, controller):
        self.controllers.append(controller)

    def __repr__(self):
        return f"Body(position={self.position}, velocity={self.velocity}, acceleration={self.acceleration}, mass={self.mass})"

class GravitationalForce:
    """Uniform gravity."""
    def __init__(self, g=GRAVITY):
        self.g = g

    def apply_to(self, body):
        body.apply_force(Vector3D(0, 0, -self.g * body.mass))

class GroundCollision:
    """Simple ground collision with restitution."""
    def __init__(self, ground_level=0.0, restitution=0.5):
        self.ground_level = ground_level
        self.restitution = restitution

    def apply_to(self, body):
        half_h = getattr(body, 'half_height', 0.0)
        min_z = self.ground_level + half_h
        z = body.position.v[2]
        if z < min_z:
            body.position.v[2] = min_z
            vz = body.velocity.v[2]
            if vz < 0:
                body.velocity.v[2] = -vz * self.restitution

# --- End dynamics relocation ---

# --- Sensors, actuators, mixers, and vehicle classes relocated from components.py ---
class IMUSensor(Sensor):
    """IMU sensor model: returns noisy acceleration and angular velocity."""
    def __init__(self, accel_noise_std=0.0, gyro_noise_std=0.0,
                 accel_bias_rw_std=0.0, gyro_bias_rw_std=0.0):
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std

        self.accel_bias_rw_std = accel_bias_rw_std
        self.gyro_bias_rw_std = gyro_bias_rw_std

        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)

    def read(self, objects, dt):
        for obj in objects:
            true_accel = obj.acceleration.v
            true_gyro = obj.angular_velocity.v

            if self.accel_bias_rw_std > 0.0:
                self._accel_bias += np.random.randn(3) * self.accel_bias_rw_std * np.sqrt(dt)
            if self.gyro_bias_rw_std > 0.0:
                self._gyro_bias += np.random.randn(3) * self.gyro_bias_rw_std * np.sqrt(dt)

            noisy_accel = true_accel + self._accel_bias + np.random.randn(3) * self.accel_noise_std
            noisy_gyro = true_gyro + self._gyro_bias + np.random.randn(3) * self.gyro_noise_std

            obj.sensor_data = {
                'accel': noisy_accel,
                'gyro': noisy_gyro,
                'accel_bias': self._accel_bias.copy(),
                'gyro_bias': self._gyro_bias.copy(),
            }

class Motor(Actuator):
    """Individual rotor model with first-order lag and noise."""
    def __init__(self, idx, r_body, spin, kQ=0.02, thrust_noise_std=0.0, torque_noise_std=0.0, tau=0.02, max_thrust=None):
        self.idx = idx
        self.r_body = r_body
        self.spin = spin
        self.kQ = kQ
        self.thrust_noise_std = thrust_noise_std
        self.torque_noise_std = torque_noise_std
        self.tau = tau
        self._thrust_state = 0.0
        self.max_thrust = max_thrust

    def apply_to(self, obj, dt):
        thrusts = getattr(obj, 'motor_thrusts', None)
        if thrusts is None or self.idx >= len(thrusts):
            return
        desired = thrusts[self.idx]
        if self.max_thrust is not None:
            desired = min(desired, self.max_thrust)
        self._thrust_state += (desired - self._thrust_state)*(dt/self.tau)
        thrust = self._thrust_state
        if self.max_thrust is not None and thrust > self.max_thrust:
            thrust = self.max_thrust
        if self.thrust_noise_std > 0.0:
            thrust += np.random.randn()*self.thrust_noise_std
            thrust = max(0.0, thrust)
        force_body = Vector3D(0,0,thrust)
        reaction = Vector3D(0,0,self.spin*self.kQ*thrust)
        arm_torque = self.r_body.cross(force_body)
        total_torque = arm_torque + reaction
        if self.torque_noise_std > 0.0:
            noise = Vector3D(*(np.random.randn(3)*self.torque_noise_std))
            total_torque += noise
        world_force = obj.orientation.rotate(force_body)
        obj.apply_force(world_force)
        obj.apply_torque(total_torque, dt)

# Add Quadcopter class
class Quadcopter(Body):
    def __init__(self, position=None, mass=1.0, arm_length=0.3):
        num_arms = 4
        m_arm = mass / num_arms
        L = arm_length
        I_z = num_arms * (1/12) * m_arm * (L**2)
        L_proj = L * np.cos(np.pi / 4)
        I_x = num_arms * (1/3) * m_arm * (L_proj**2)
        I_y = I_x
        inertia = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])
        super().__init__(position=position or Vector3D(0, 0, 0.1),
                         mass=mass, inertia=inertia)
        self.arm_length = arm_length

# Add GraspActuator class
class GraspActuator(Actuator):
    """Attaches a box rigidly to the quad at a fixed offset."""
    def __init__(self, box, offset=Vector3D(0, 0, -0.1)):
        self.box = box
        self.offset = offset

    def apply_to(self, obj, dt):
        # place box at relative offset in body frame for rigid attachment
        world_offset = obj.orientation.rotate(self.offset)
        self.box.position = obj.position + world_offset
        self.box.velocity = obj.velocity
        # match quad's orientation and angular velocity (rigid attachment)
        self.box.orientation = obj.orientation
        self.box.angular_velocity = obj.angular_velocity
        # disable gravity for carried box
        self.box.mass = self.box.mass  # keep mass
        # no actuators or controllers on box while carried 