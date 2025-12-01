"""Launch only the floating body for visualization."""

from __future__ import annotations

import pathlib
import sys

from isaac_path import ensure_isaac_python_path

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

ensure_isaac_python_path()

from omni.isaac.core import World
from omni.isaac.kit import SimulationApp

from extensions.floating_body.scripts.floater_setup import spawn_floater
from extensions.floating_body.scripts.wave_forcing import airy_wave_elevation


def main():
    sim_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
    world = World(stage_units_in_meters=1.0)

    floater = spawn_floater(world=world)
    world.reset()

    time = 0.0
    dt = 0.01
    while time < 1.0:
        elevation = airy_wave_elevation(time, amplitude=0.1, frequency=0.2)
        floater.set_world_pose([0.0, 0.0, 0.5 + elevation])
        world.step(render=True)
        time += dt

    sim_app.close()


if __name__ == "__main__":
    main()
