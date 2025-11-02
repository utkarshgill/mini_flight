"""Test GenericMixer correctness for a plus quad configuration."""

import numpy as np
from common.math import Vector3D
from miniflight.control import GenericMixer


class Dummy:
    pass


def test_generic_mixer_inverts_plus_config():
    L = 0.3
    motor_positions = [Vector3D( L, 0, 0), Vector3D(0,  L, 0), Vector3D(-L, 0, 0), Vector3D(0, -L, 0)]
    spins = [1, -1, 1, -1]
    mixer = GenericMixer(motor_positions, spins, kT=1.0, kQ=0.02)

    cmd = np.array([10.0, 0.2, -0.1, 0.05])  # [T, τx, τy, τz]

    dummy = Dummy()
    dummy.control_command = cmd
    mixer.apply_to(dummy, dt=0.01)

    thrusts = dummy.motor_thrusts

    # Forward compute produced [T, τx, τy, τz]
    A = mixer._A  # 4 x 4
    produced = A @ thrusts

    assert np.allclose(produced, cmd, atol=1e-6), f"Produced {produced} != command {cmd}" 