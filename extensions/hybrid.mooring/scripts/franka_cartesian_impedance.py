"""Simplified Cartesian impedance for a Franka arm.

The implementation is intentionally high level; it shows how impedance
and feedforward wrenches can be combined before translating to joint
torques through an articulation controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass
class ImpedanceGains:
    translational_stiffness: np.ndarray
    translational_damping: np.ndarray
    rotational_stiffness: np.ndarray
    rotational_damping: np.ndarray


class FrankaCartesianImpedance:
    def __init__(self, gains: ImpedanceGains):
        self.gains = gains
        self.desired_pose = np.zeros(6)
        self.desired_twist = np.zeros(6)

    def set_desired(self, pose: Iterable[float], twist: Iterable[float]) -> None:
        pose_vec = np.asarray(pose, dtype=float)
        twist_vec = np.asarray(twist, dtype=float)
        if pose_vec.shape != (6,) or twist_vec.shape != (6,):
            raise ValueError("pose and twist must be length-6")
        self.desired_pose = pose_vec
        self.desired_twist = twist_vec

    def step(self, current_pose: Iterable[float], current_twist: Iterable[float], feedforward_wrench: Iterable[float]) -> np.ndarray:
        pose_vec = np.asarray(current_pose, dtype=float)
        twist_vec = np.asarray(current_twist, dtype=float)
        wrench_vec = np.asarray(feedforward_wrench, dtype=float)

        pose_error = self.desired_pose - pose_vec
        twist_error = self.desired_twist - twist_vec

        stiffness = np.concatenate((self.gains.translational_stiffness, self.gains.rotational_stiffness))
        damping = np.concatenate((self.gains.translational_damping, self.gains.rotational_damping))

        impedance_wrench = stiffness * pose_error + damping * twist_error
        return impedance_wrench + wrench_vec
