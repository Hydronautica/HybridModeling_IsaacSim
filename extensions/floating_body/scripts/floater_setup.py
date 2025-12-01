"""Create a simple floater prim for the hybrid simulation."""

from __future__ import annotations

from typing import Optional, Sequence

from omni.isaac.core.objects import DynamicCuboid


def spawn_floater(
    size: Sequence[float] = (1.0, 1.0, 0.5),
    position: Sequence[float] = (0.0, 0.0, 0.5),
    world: Optional[object] = None,
) -> DynamicCuboid:
    """Create the floater prim and optionally register it with a ``World`` scene."""

    floater = DynamicCuboid(
        prim_path="/World/Floater",
        name="floater",
        size=list(size),
        position=list(position),
    )

    if world is not None:
        world.scene.add(floater)

    return floater
