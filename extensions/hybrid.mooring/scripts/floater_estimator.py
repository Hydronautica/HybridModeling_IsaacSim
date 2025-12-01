"""Helpers for reading floater state inside Isaac Sim."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class FloaterState:
    position: np.ndarray
    orientation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray

    def as_twist(self) -> np.ndarray:
        return np.concatenate([self.linear_velocity, self.angular_velocity])


class FloaterEstimator:
    """Thin wrapper around an Isaac Sim prim for easier mocking."""

    def __init__(self, floater):
        self._floater = floater

    def get_state(self) -> FloaterState:
        pos, quat = self._floater.get_world_pose()
        lin_vel = self._floater.get_linear_velocity()
        ang_vel = self._floater.get_angular_velocity()
        return FloaterState(
            position=np.asarray(pos, dtype=float),
            orientation=np.asarray(quat, dtype=float),
            linear_velocity=np.asarray(lin_vel, dtype=float),
            angular_velocity=np.asarray(ang_vel, dtype=float),
        )


def get_floater_state(floater) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Functional interface mirroring the estimator class."""

    estimator = FloaterEstimator(floater)
    state = estimator.get_state()
    return state.position, state.orientation, state.linear_velocity, state.angular_velocity
