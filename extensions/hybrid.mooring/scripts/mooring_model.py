"""Lightweight numerical mooring model utilities.

The functions are deliberately simple so they can be swapped for
higher-fidelity solvers (catenary, Morison, or BEM) without changing
call sites in the hybrid controller.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Dict, Iterable

import numpy as np
import yaml


def _axis_vector(params: Dict[str, float]) -> np.ndarray:
    return np.array(
        [params[axis] for axis in ("surge", "sway", "heave", "roll", "pitch", "yaw")],
        dtype=float,
    )


@dataclass
class MooringParameters:
    stiffness: np.ndarray
    damping: np.ndarray

    @classmethod
    def from_yaml(cls, path: pathlib.Path) -> "MooringParameters":
        data = yaml.safe_load(path.read_text())
        return cls(_axis_vector(data["stiffness"]), _axis_vector(data["damping"]))


def compute_mooring_force(position: Iterable[float], velocity: Iterable[float], params: MooringParameters) -> np.ndarray:
    """Compute a 6D wrench from displacement and velocity.

    The returned vector is ordered `[Fx, Fy, Fz, Mx, My, Mz]` and can
    be mapped to the end-effector frame before being forwarded to an
    impedance controller.
    """

    pos_vec = np.asarray(position, dtype=float)
    vel_vec = np.asarray(velocity, dtype=float)
    if pos_vec.shape != (6,) or vel_vec.shape != (6,):
        raise ValueError("position and velocity must be length-6 vectors")

    spring_term = -params.stiffness * pos_vec
    damping_term = -params.damping * vel_vec
    return spring_term + damping_term


def combine_wrenches(*wrenches: Iterable[float]) -> np.ndarray:
    """Sum multiple 6D wrenches with shape validation."""

    total = np.zeros(6, dtype=float)
    for wrench in wrenches:
        arr = np.asarray(wrench, dtype=float)
        if arr.shape != (6,):
            raise ValueError("All wrenches must be length-6")
        total += arr
    return total
