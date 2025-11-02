"""
Human Interface Layer (HIL)
Provides user input abstractions for RC, gamepad, etc.
"""
from common.interface import Controller
import numpy as np
try:
    import hid
except ImportError:
    hid = None
from common.math import wrap_angle
# PS5 DualSense constants
VENDOR_ID = 0x054C
PRODUCT_ID = 0x0CE6
REPORT_ID = 0x01
from miniflight.control import StabilityController

class HIL:
    """
    Base class for human-in-the-loop input devices.
    """
    def read_input(self) -> dict:
        """
        Return a dictionary of control inputs.
        """
        raise NotImplementedError

class Keyboard(HIL):
    """
    Keyboard and PS5 cross input HIL stub for gym environments.
    """
    def __init__(self):
        # Initialize key state and cross button tracking
        self.key_state = {}
        self.cross_pressed = False
        self.cross_handled = False
        # Set control gains for keyboard input
        self.thrust_gain = 5.0
        self.max_tilt = 0.5
        self.yaw_rate_gain = np.pi/2

    def update_key_state(self, key_state):
        """Update key_state from renderer-provided input."""
        self.key_state.clear()
        if isinstance(key_state, dict):
            for key, pressed in key_state.items():
                if pressed:
                    self.key_state[key] = True
        else:
            for key in key_state:
                self.key_state[key] = True

    def on_key_press(self, event):
        """Matplotlib key press handler."""
        key = event.key
        self.key_state[key] = True

    def on_key_release(self, event):
        """Matplotlib key release handler."""
        key = event.key
        self.key_state[key] = False

    def update(self, stability_ctrl, dt):
        # Map keyboard state to setpoints
        if self.key_state.get('w', False):
            stability_ctrl.z_setpoint += self.thrust_gain * dt
        if self.key_state.get('s', False):
            stability_ctrl.z_setpoint -= self.thrust_gain * dt
        stability_ctrl.z_setpoint = float(np.clip(stability_ctrl.z_setpoint, 0.0, 20.0))
        # Roll and pitch
        roll_t = (-self.max_tilt if self.key_state.get('left', False)
                  else self.max_tilt if self.key_state.get('right', False)
                  else 0.0)
        pitch_t = (self.max_tilt if self.key_state.get('up', False)
                   else -self.max_tilt if self.key_state.get('down', False)
                   else 0.0)
        # Yaw
        yaw_sp = stability_ctrl.yaw_setpoint
        if self.key_state.get('a', False):
            yaw_sp += self.yaw_rate_gain * dt
        if self.key_state.get('d', False):
            yaw_sp -= self.yaw_rate_gain * dt
        yaw_sp = wrap_angle(yaw_sp)
        stability_ctrl.set_attitude_target(roll_t, pitch_t, yaw_sp)

class DualSense(HIL):
    """
    Reads PS5 DualSense sticks and keyboard fallback, drives StabilityController's PID loops.
    """
    def __init__(self, thrust_gain=5.0, max_tilt_rad=0.5, yaw_rate_gain=np.pi/2):
        self.thrust_gain = thrust_gain
        self.max_tilt = max_tilt_rad
        self.yaw_rate_gain = yaw_rate_gain
        # HID device for DualSense
        self.h = hid.device() if hid else None
        if self.h:
            try:
                self.h.open(VENDOR_ID, PRODUCT_ID)
                self.h.set_nonblocking(True)
            except Exception:
                self.h = None
        # Initialize state tracking
        self.key_state = {}
        self.cross_pressed = False
        self.cross_handled = False

    def update(self, stability_ctrl, dt):
        # Read PS5 input if available and update stability controller setpoints
        if self.h:
            try:
                data = self.h.read(64)
            except OSError:
                data = None
            if data and data[0] == REPORT_ID:
                # Update cross button and stick inputs
                self.cross_pressed = bool(data[8] & 0x20)
                lx, ly, rx, ry = data[1], data[2], data[3], data[4]
                def deadzone(v, dz=0.1): return v if abs(v) > dz else 0.0
                norm_lx = deadzone((lx - 127)/127.0)
                norm_ly = deadzone((127 - ly)/127.0)
                norm_rx = deadzone((rx - 127)/127.0)
                norm_ry = deadzone((127 - ry)/127.0)
                # Altitude setpoint
                stability_ctrl.z_setpoint += norm_ly * self.thrust_gain * dt
                stability_ctrl.z_setpoint = float(np.clip(stability_ctrl.z_setpoint, 0.0, 20.0))
                # Attitude setpoints
                stability_ctrl.set_attitude_target(
                    np.clip(norm_rx * self.max_tilt, -self.max_tilt, self.max_tilt),
                    np.clip(norm_ry * self.max_tilt, -self.max_tilt, self.max_tilt),
                    wrap_angle(stability_ctrl.yaw_setpoint - norm_lx * self.yaw_rate_gain * dt)
                )
        # No direct control output; stability controller drives actuators 