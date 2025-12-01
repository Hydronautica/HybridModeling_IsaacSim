"""Minimal buttons for running the hybrid simulation."""

from __future__ import annotations

from typing import Callable

import omni.ui as ui


def build_controls(on_reset: Callable[[], None], on_toggle: Callable[[], None]) -> None:
    with ui.HStack(height=40):
        ui.Button("Reset", clicked_fn=on_reset)
        ui.Button("Start/Stop", clicked_fn=on_toggle)
