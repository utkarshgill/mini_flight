import unittest

import numpy as np
from sim.engine import GravitationalForce, EulerIntegrator, Body
from common.math import Vector3D
from scipy.spatial.transform import Rotation as SciRot
from scipy.integrate import solve_ivp

class TestPhysics(unittest.TestCase):
    def test_gravity_acceleration(self):
        body = Body(mass=2.0)
        gf = GravitationalForce(g=9.8)
        gf.apply_to(body)
        # acceleration should be (0,0,-9.8)
        self.assertTrue(np.allclose(body.acceleration.v, [0, 0, -9.8], atol=1e-6))

    def test_euler_integrator_linear_integration(self):
        body = Body()
        body.acceleration = Vector3D(1, 0, 0)
        body.velocity = Vector3D(0, 0, 0)
        body.position = Vector3D(0, 0, 0)
        integrator = EulerIntegrator(angular_damp=1.0, linear_drag=1.0)
        integrator.step(body, dt=1.0)
        # After one second: velocity = 1, position = 1
        self.assertTrue(np.allclose(body.velocity.v, [1, 0, 0], atol=1e-6))
        self.assertTrue(np.allclose(body.position.v, [1, 0, 0], atol=1e-6))

    def test_euler_integrator_angular_integration(self):
        # Verify quaternion integration against SciPy ground truth for constant angular velocity
        body = Body()
        # disable linear motion
        body.acceleration = Vector3D()
        body.velocity = Vector3D()
        body.position = Vector3D()
        # set angular velocity about Z axis
        omega = 0.5
        body.angular_velocity = Vector3D(0, 0, omega)
        # integrator without damping or drag
        integrator = EulerIntegrator(angular_damp=1.0, linear_drag=1.0)
        dt = 0.01
        integrator.step(body, dt=dt)
        # actual rotation matrix from integrator
        actual = body.orientation.as_rotation_matrix()
        # expected rotation about Z by omega*dt
        expected = SciRot.from_rotvec([0, 0, omega * dt]).as_matrix()
        self.assertTrue(np.allclose(actual, expected, atol=1e-4))

    def test_euler_integrator_linear_vs_scipy(self):
        # Compare EulerIntegrator linear step against SciPy solve_ivp for constant acceleration
        a = 1.0
        dt = 1e-3
        # SciPy ground truth integration of y'=[v, a]
        def f(t, y):
            return [y[1], a]
        sol = solve_ivp(f, [0, dt], [0, 0], t_eval=[dt])
        expected_pos, expected_vel = sol.y[:, 0]

        body = Body()
        body.acceleration = Vector3D(a, 0, 0)
        body.velocity = Vector3D(0, 0, 0)
        body.position = Vector3D(0, 0, 0)
        integrator = EulerIntegrator(angular_damp=1.0, linear_drag=1.0)
        integrator.step(body, dt=dt)
        actual_vel = body.velocity.v[0]
        actual_pos = body.position.v[0]
        self.assertTrue(np.allclose(actual_vel, expected_vel, atol=1e-6))
        self.assertTrue(np.allclose(actual_pos, expected_pos, atol=1e-6))

if __name__ == '__main__':
    unittest.main() 