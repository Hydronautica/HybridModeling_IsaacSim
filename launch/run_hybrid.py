"""Run the hybrid mooring controller loop inside Isaac Sim."""

from __future__ import annotations

import pathlib
import sys

import numpy as np

from isaac_path import ensure_isaac_python_path

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

ensure_isaac_python_path()

from omni.isaac.core import World
from omni.isaac.kit import SimulationApp

from extensions.floating_body.scripts.floater_setup import spawn_floater
from extensions.hybrid.mooring.scripts.drag_grab_enable import enable_mouse_grab
from extensions.hybrid.mooring.scripts.franka_cartesian_impedance import ImpedanceGains
from extensions.hybrid.mooring.scripts.hybrid_controller import HybridConfig, HybridController
from extensions.hybrid.mooring.scripts.mooring_model import MooringParameters


class DummyFrankaController:
    """Placeholder articulation control surface."""

    def apply_action(self, joint_efforts: np.ndarray, dt: float) -> None:
        print(f"Applying torques {joint_efforts} with dt={dt:.3f}")


def main():
    sim_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
    world = World(stage_units_in_meters=1.0)

    floater = spawn_floater(world=world)
    enable_mouse_grab()

    world.reset()

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
    while sim_time < 0.05:
        hybrid.step(dt)
        world.step(render=True)
        sim_time += dt

    sim_app.close()


if __name__ == "__main__":
    main()
