"""Launch only the floating body for visualization."""

from __future__ import annotations

import pathlib
import sys

from isaac_path import ensure_isaac_python_path

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

ensure_isaac_python_path()

from extensions.floating_body.scripts.floater_setup import spawn_floater
from extensions.floating_body.scripts.wave_forcing import airy_wave_elevation


def main():
    floater = spawn_floater()
    time = 0.0
    dt = 0.01
    while time < 1.0:
        elevation = airy_wave_elevation(time, amplitude=0.1, frequency=0.2)
        floater.set_world_pose([0.0, 0.0, 0.5 + elevation])
        time += dt


if __name__ == "__main__":
    main()
