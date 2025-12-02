"""Create a Mujoco floater for the hybrid simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

import mujoco
import numpy as np

MJCF_TEMPLATE = """
<mujoco model="floater">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true" />
  <option timestep="{timestep}" gravity="0 0 -9.81" density="1000" />
  <visual>
    <scale forcewidth="0.02" />
  </visual>
  <worldbody>
    <geom name="ocean" type="plane" size="5 5 0.1" rgba="0.2 0.3 0.4 1" />
    <body name="floater" pos="{x} {y} {z}">
      <freejoint />
      <geom name="hull" type="box" size="{sx} {sy} {sz}" density="600" rgba="0.8 0.3 0.3 1" />
    </body>
  </worldbody>
</mujoco>
"""


@dataclass
class MujocoFloater:
    """Lightweight wrapper exposing the accessor surface expected by the hybrid controller."""

    model: mujoco.MjModel
    data: mujoco.MjData
    body_id: int

    def set_world_pose(self, position: Iterable[float], orientation: Iterable[float] | None = None) -> None:
        pos = np.asarray(position, dtype=float)
        quat = np.asarray(orientation if orientation is not None else [1.0, 0.0, 0.0, 0.0], dtype=float)
        self.data.qpos[:3] = pos
        self.data.qpos[3:7] = quat
        mujoco.mj_forward(self.model, self.data)

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.data.qpos[:3].copy(), self.data.qpos[3:7].copy()

    def get_linear_velocity(self) -> np.ndarray:
        return self.data.qvel[:3].copy()

    def get_angular_velocity(self) -> np.ndarray:
        return self.data.qvel[3:6].copy()

    def apply_wrench(self, wrench: Iterable[float]) -> None:
        wrench_vec = np.asarray(wrench, dtype=float)
        if wrench_vec.shape != (6,):
            raise ValueError("wrench must be length-6")
        self.data.xfrc_applied[self.body_id] = wrench_vec

    def clear_applied_forces(self) -> None:
        self.data.xfrc_applied[:] = 0.0

    def step(self, dt: float) -> None:
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.model.opt.timestep = dt
        mujoco.mj_step(self.model, self.data)
        self.clear_applied_forces()


def spawn_floater(
    size: Sequence[float] = (0.5, 0.5, 0.25),
    position: Sequence[float] = (0.0, 0.0, 0.5),
    timestep: float = 0.01,
) -> MujocoFloater:
    xml = MJCF_TEMPLATE.format(
        timestep=timestep,
        x=position[0],
        y=position[1],
        z=position[2],
        sx=size[0] / 2,
        sy=size[1] / 2,
        sz=size[2] / 2,
    )
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "floater")
    floater = MujocoFloater(model=model, data=data, body_id=body_id)
    floater.set_world_pose(position)
    return floater
