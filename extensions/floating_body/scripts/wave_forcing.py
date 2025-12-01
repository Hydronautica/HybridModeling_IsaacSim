"""Simple wave forcing utilities."""

from __future__ import annotations

import numpy as np


def airy_wave_elevation(time: float, amplitude: float, frequency: float, phase: float = 0.0) -> float:
    return amplitude * np.sin(2.0 * np.pi * frequency * time + phase)


def morison_inline_force(velocity: float, diameter: float, rho: float = 1025.0, drag_coefficient: float = 1.0) -> float:
    return 0.5 * rho * drag_coefficient * np.pi * (0.25 * diameter ** 2) * velocity * abs(velocity)
