"""Create a simple floater prim for the hybrid simulation."""

from __future__ import annotations

from typing import Sequence

from omni.isaac.core.objects import DynamicCuboid


def spawn_floater(size: Sequence[float] = (1.0, 1.0, 0.5), position: Sequence[float] = (0.0, 0.0, 0.5)) -> DynamicCuboid:
    return DynamicCuboid(
        prim_path="/World/Floater",
        name="floater",
        size=list(size),
        position=list(position),
    )
