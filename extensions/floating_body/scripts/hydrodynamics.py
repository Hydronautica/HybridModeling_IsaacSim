"""Placeholder hydrodynamic loads."""

from __future__ import annotations

from typing import Iterable

import numpy as np


def radiation_damping(velocity: Iterable[float], coeff: float = 0.0) -> np.ndarray:
    vel_vec = np.asarray(velocity, dtype=float)
    if vel_vec.shape != (6,):
        raise ValueError("velocity must be length-6")
    return -coeff * vel_vec


def add_restoring(restoring_matrix: np.ndarray, displacement: Iterable[float]) -> np.ndarray:
    disp_vec = np.asarray(displacement, dtype=float)
    return restoring_matrix @ disp_vec
