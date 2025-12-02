"""Hybrid numerical–physical mooring controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .floater_estimator import FloaterEstimator, FloaterState
from .franka_cartesian_impedance import FrankaCartesianImpedance, ImpedanceGains
from .mooring_model import MooringParameters, combine_wrenches, compute_mooring_force


@dataclass
class HybridConfig:
    mooring_params: MooringParameters
    franka_gains: ImpedanceGains


@dataclass
class HybridStepResult:
    state: FloaterState
    mooring_wrench: np.ndarray
    end_effector_wrench: np.ndarray
    commanded_wrench: np.ndarray
    joint_torques: np.ndarray


class HybridController:
    def __init__(self, floater, franka_controller, config: HybridConfig):
        self.floater_estimator = FloaterEstimator(floater)
        self.franka_impedance = FrankaCartesianImpedance(config.franka_gains)
        self.franka_controller = franka_controller
        self.config = config

    def map_force_to_ee(self, floater_wrench: Iterable[float]) -> np.ndarray:
        """Placeholder frame transform for end-effector mapping."""

        wrench_vec = np.asarray(floater_wrench, dtype=float)
        return wrench_vec

    def step(self, dt: float) -> HybridStepResult:
        state = self.floater_estimator.get_state()
        floater_twist = state.as_twist()
        floater_disp = np.concatenate((state.position, np.zeros(3)))

        mooring_wrench = compute_mooring_force(floater_disp, floater_twist, self.config.mooring_params)
        end_effector_wrench = self.map_force_to_ee(mooring_wrench)

        commanded_wrench = combine_wrenches(end_effector_wrench)
        torques = self.franka_impedance.step(state.position, floater_twist, commanded_wrench)

        self.franka_controller.apply_action(torques, dt)
        return HybridStepResult(
            state=state,
            mooring_wrench=mooring_wrench,
            end_effector_wrench=end_effector_wrench,
            commanded_wrench=commanded_wrench,
            joint_torques=torques,
        )
