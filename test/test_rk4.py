"""Validate Runge–Kutta integrator against SciPy's solve_ivp for free fall.

The test passes if the simulated position after 1 s of free‐fall matches the
reference solution within 1 µm.  If SciPy is unavailable it falls back to the
analytic solution z = −½ g t².
"""

import importlib
import math

import numpy as np
import pytest

from sim.engine import Body, RungeKuttaIntegrator
from common.math import Vector3D


@pytest.mark.parametrize("dt", [1e-3, 5e-3, 1e-2])
def test_free_fall_matches_reference(dt: float):
    g = 9.81
    T = 1.0  # total simulation time (s)
    steps = int(T / dt)

    # --- Sim using our RK4 integrator -----------------------------------
    quad = Body(
        position=Vector3D(0, 0, 0),
        velocity=Vector3D(0, 0, 0),
        mass=1.0,
        integrator=RungeKuttaIntegrator(angular_damp=1.0, linear_drag=1.0),
    )
    quad.acceleration = Vector3D(0, 0, -g)

    for _ in range(steps):
        quad.update(dt)
        # constant acceleration; re-apply each step since integrator resets it
        quad.acceleration = Vector3D(0, 0, -g)

    z_sim = quad.position.v[2]

    # --- Reference solution ---------------------------------------------
    scipy_spec = importlib.util.find_spec("scipy.integrate")
    if scipy_spec is not None:
        from scipy.integrate import solve_ivp

        def ode(t, y):
            pos, vel = y
            return [vel, -g]

        sol = solve_ivp(ode, [0, T], [0.0, 0.0], t_eval=[T])
        z_ref = sol.y[0, -1]
    else:
        # Analytic free-fall
        z_ref = -0.5 * g * T ** 2

    # --------------------------------------------------------------------
    assert math.isclose(z_sim, z_ref, rel_tol=0, abs_tol=1e-6), f"sim={z_sim}, ref={z_ref}" 