"""Launch only the Mujoco floating body for visualization/testing."""

from __future__ import annotations

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from extensions.floating_body.scripts.floater_setup import spawn_floater
from extensions.floating_body.scripts.wave_forcing import airy_wave_elevation


def main():
    floater = spawn_floater()
    time = 0.0
    dt = 0.01
    nominal_height = floater.get_world_pose()[0][2]
    heave_stiffness = 400.0
    heave_damping = 50.0

    while time < 2.0:
        elevation = airy_wave_elevation(time, amplitude=0.1, frequency=0.2)
        desired_z = nominal_height + elevation

        current_pos, _ = floater.get_world_pose()
        vertical_error = desired_z - current_pos[2]
        vertical_velocity = floater.get_linear_velocity()[2]

        heave_force = heave_stiffness * vertical_error - heave_damping * vertical_velocity
        floater.apply_wrench([0.0, 0.0, heave_force, 0.0, 0.0, 0.0])
        floater.step(dt)
        time += dt


if __name__ == "__main__":
    main()
