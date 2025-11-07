"""
Control module: generic PID and stability controllers.
"""
import numpy as np

from common.interface import Controller, Actuator
from common.math import wrap_angle, GRAVITY, Quaternion
from common.types import StateEstimate


 

class PIDController:
    """
    Generic PID controller: tracks error and computes control outputs.
    Accepts pre-computed error, does not handle setpoints or angle wrapping.
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        """Clear integral and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error, dt):
        """
        Compute PID output given an error and timestep dt.
        """
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error
        return output

class StabilityController(Controller):
    """
    Handles core stabilization PID loops for position and attitude.
    Wraps multiple PIDController instances for x, y, z, roll, pitch, yaw.
    """

    def __init__(self):
        # Default gains (tunable)
        self.x_pid = PIDController(kp=0.2, ki=0.0, kd=0.3)
        self.y_pid = PIDController(kp=0.2, ki=0.0, kd=0.3)
        self.z_pid = PIDController(kp=1.5, ki=0.2, kd=3.0)
        self.roll_pid = PIDController(kp=2.0, ki=0.0, kd=0.3)
        self.pitch_pid = PIDController(kp=2.0, ki=0.0, kd=0.3)
        self.yaw_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        # Reset integral and previous error state for all PIDs
        for pid in (self.x_pid, self.y_pid, self.z_pid, self.roll_pid, self.pitch_pid, self.yaw_pid):
            pid.reset()
        # Initialize setpoints for each loop
        self.x_setpoint = 0.0
        self.y_setpoint = 0.0
        self.z_setpoint = 1.0  # Default hover altitude
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0

    def update(self, state: StateEstimate, dt):
        # Position errors
        x_error = self.x_setpoint - state.position.v[0]
        y_error = self.y_setpoint - state.position.v[1]
        z_error = self.z_setpoint - state.position.v[2]
        # Attitude error via quaternion (avoiding Euler singularities)
        q_current = state.orientation
        q_desired = Quaternion.from_euler(self.roll_setpoint, self.pitch_setpoint, self.yaw_setpoint)
        q_error = q_desired * q_current.conjugate()
        q_error.normalize()
        # Axis-angle error vector in inertial frame
        rot_vec = q_error.to_rotation_vector()
        # Express error in body axes by rotating into local frame
        err_body = q_current.conjugate().rotate(rot_vec)
        roll_error, pitch_error, yaw_error = err_body.v.tolist()
        # Update PIDs with pre-computed errors
        pitch_cmd = float(np.clip(self.x_pid.update(x_error, dt), -0.3, 0.3))
        roll_cmd = float(np.clip(-self.y_pid.update(y_error, dt), -0.3, 0.3))
        thrust_baseline = GRAVITY + self.z_pid.update(z_error, dt)
        thrust_cmd = float(np.clip(thrust_baseline, 0.0, 20.0))
        # Inner-loop attitude
        roll_final = float(np.clip(self.roll_pid.update(roll_error, dt), -0.5, 0.5))
        pitch_final = float(np.clip(self.pitch_pid.update(pitch_error, dt), -0.5, 0.5))
        yaw_final = float(np.clip(self.yaw_pid.update(yaw_error, dt), -0.3, 0.3))
        return [thrust_cmd, roll_final, pitch_final, yaw_final]

    def set_xyz_target(self, x=None, y=None, z=None):
        """Dynamically update horizontal XYZ setpoints via unified API."""
        if x is not None:
            self.x_setpoint = x
        if y is not None:
            self.y_setpoint = y
        if z is not None:
            self.z_setpoint = z

    def set_attitude_target(self, roll=None, pitch=None, yaw=None):
        """Dynamically update attitude setpoints via unified API."""
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