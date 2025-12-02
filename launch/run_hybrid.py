"""Run the hybrid mooring controller loop inside Mujoco."""

from __future__ import annotations

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from extensions.floating_body.scripts.floater_setup import MujocoFloater, spawn_floater
from extensions.hybrid.mooring.scripts.franka_cartesian_impedance import ImpedanceGains
from extensions.hybrid.mooring.scripts.hybrid_controller import HybridConfig, HybridController
from extensions.hybrid.mooring.scripts.mooring_model import MooringParameters


class DummyFrankaController:
    """Placeholder articulation control surface."""

    def apply_action(self, joint_efforts: np.ndarray, dt: float) -> None:
        print(f"Applying torques {joint_efforts} with dt={dt:.3f}")


def main():
    floater: MujocoFloater = spawn_floater()

    config = HybridConfig(
        mooring_params=MooringParameters.from_yaml(
            REPO_ROOT / "extensions" / "hybrid.mooring" / "config" / "mooring_params.yaml"
        ),
        franka_gains=ImpedanceGains(
            translational_stiffness=np.array([400.0, 400.0, 600.0]),
            translational_damping=np.array([50.0, 50.0, 70.0]),
            rotational_stiffness=np.array([25.0, 25.0, 25.0]),
            rotational_damping=np.array([3.5, 3.5, 3.5]),
        ),
    )

    hybrid = HybridController(floater, DummyFrankaController(), config)

    dt = 0.01
    sim_time = 0.0
    while sim_time < 1.0:
        step_result = hybrid.step(dt)
        floater.apply_wrench(step_result.mooring_wrench)
        floater.step(dt)
        sim_time += dt


if __name__ == "__main__":
    main()
