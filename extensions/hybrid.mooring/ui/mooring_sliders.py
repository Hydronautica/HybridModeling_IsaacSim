"""Tiny UI sketch for tuning mooring gains at runtime."""

from __future__ import annotations

from typing import Callable, Dict

import omni.ui as ui


class MooringSliderPanel:
    def __init__(self, on_update: Callable[[str, float], None]):
        self.on_update = on_update

    def build(self, initial: Dict[str, float]) -> None:
        with ui.VStack():
            ui.Label("Mooring stiffness/damping")
            for key, value in initial.items():
                with ui.HStack():
                    ui.Label(key, width=120)
                    ui.FloatSlider(min=0.0, max=8000.0, value=value).model.add_value_changed_fn(
                        lambda model, k=key: self.on_update(k, model.as_float)
                    )
