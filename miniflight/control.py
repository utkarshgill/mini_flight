"""
Control module: generic PID and stability controllers.
"""
import numpy as np

from common.interface import Controller, Actuator
from common.math import wrap_angle, GRAVITY, Quaternion
from common.types import StateEstimate


 

class PIDController:
    """Generic PID controller that clamps its integral term."""

    def __init__(self, kp, ki, kd, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = float(abs(integral_limit))
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error, dt):
        self._integral += error * dt
        self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error
        return output

class StabilityController(Controller):
    """Altitude hold + angle controller."""

    def __init__(self):
        self.z_pid = PIDController(kp=1.2, ki=0.1, kd=2.5)
        self.roll_pid = PIDController(kp=4.5, ki=0.7, kd=0.65, integral_limit=1.5)
        self.pitch_pid = PIDController(kp=4.5, ki=0.7, kd=0.65, integral_limit=1.5)
        self.yaw_pid = PIDController(kp=1.5, ki=0.0, kd=0.2)

        for pid in (self.z_pid, self.roll_pid, self.pitch_pid, self.yaw_pid):
            pid.reset()

        self.z_setpoint = 1.0
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0

    def update(self, state: StateEstimate, dt):
        # Altitude control (collective thrust)
        z_error = self.z_setpoint - state.position.v[2]
        thrust = GRAVITY + self.z_pid.update(z_error, dt)
        thrust = float(np.clip(thrust, 0.0, 20.0))

        # Attitude error via quaternion
        q_current = state.orientation
        q_desired = Quaternion.from_euler(self.roll_setpoint, self.pitch_setpoint, self.yaw_setpoint)
        q_error = q_desired * q_current.conjugate()
        q_error.normalize()
        rot_vec = q_error.to_rotation_vector()
        err_body = q_current.conjugate().rotate(rot_vec)
        roll_error, pitch_error, yaw_error = err_body.v.tolist()

        roll_cmd = float(np.clip(self.roll_pid.update(roll_error, dt), -0.6, 0.6))
        pitch_cmd = float(np.clip(self.pitch_pid.update(pitch_error, dt), -0.6, 0.6))
        yaw_cmd = float(np.clip(self.yaw_pid.update(yaw_error, dt), -0.4, 0.4))
        return [thrust, roll_cmd, pitch_cmd, yaw_cmd]

    def set_xyz_target(self, x=None, y=None, z=None):
        if z is not None:
            self.z_setpoint = z

    def set_attitude_target(self, roll=None, pitch=None, yaw=None):
        if roll is not None:
            self.roll_setpoint = roll
        if pitch is not None:
            self.pitch_setpoint = pitch
        if yaw is not None:
            self.yaw_setpoint = wrap_angle(yaw)

# -----------------------------------------------------------------------------
# GenericMixer moved here from targets/sim/components.py
class GenericMixer(Actuator):
    """Generic mixer for arbitrary multirotor geometry."""
    def __init__(self, motor_positions, spins, kT=1.0, kQ=0.02):
        if len(motor_positions) != len(spins):
            raise ValueError("motor_positions and spins must have same length")
        self.motor_positions = motor_positions
        self.spins = spins
        self.kT = kT
        self.kQ = kQ
        rows = []
        rows.append([kT] * len(spins))
        rows.append([pos.v[1] * kT for pos in motor_positions])
        rows.append([-pos.v[0] * kT for pos in motor_positions])
        rows.append([s * kQ for s in spins])
        self._A = np.array(rows)
        self._A_inv = np.linalg.pinv(self._A)

    def apply_to(self, obj, dt):
        cmd = getattr(obj, 'control_command', None)
        if cmd is None or len(cmd) != 4:
            return
        thrusts = np.dot(self._A_inv, np.array(cmd))
        thrusts = np.clip(thrusts, 0.0, None)
        obj.motor_thrusts = thrusts 

    def mix(self, cmd):
        """Mix a 4-channel command into per-motor thrusts without needing an obj."""
        thrusts = np.dot(self._A_inv, np.array(cmd))
        return np.clip(thrusts, 0.0, None) 