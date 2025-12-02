# Hybrid Floating Body and Mooring Control Sandbox

This repository now provides a Mujoco-based sandbox that demonstrates how to couple a numerically evaluated mooring model with a physically simulated floating body and a Franka arm applying equivalent forces. The layout mirrors research-style extensions so the system can evolve into a full hybrid experimental setup.

## Repository layout
```
hybrid_floater_system/
├── extensions/
│   ├── hybrid.mooring/        # Hybrid controller, Franka control, UI helpers
│   └── floating_body/         # Mujoco floater asset, wave and hydrodynamic helpers
├── launch/                    # Entrypoints for standalone or hybrid runs
└── docs/                      # High-level design notes and diagrams
```

## Quick start (conceptual)
1. Launch `run_hybrid.py` to spawn the floater inside Mujoco and iterate the hybrid controller loop.
2. The `HybridController` samples the floater pose and velocity, calls the numerical mooring model, maps the resulting wrench to the Franka end-effector, forwards torques to a placeholder controller, and applies the mooring wrench back onto the floater.
3. Use `run_sim.py` to exercise the floater under wave-induced heave forces without running the hybrid controller.

The scripts are intentionally lightweight: they are meant as templates for integrating real hydrodynamics, mooring solvers, and control laws. Install the [mujoco](https://pypi.org/project/mujoco/) Python package to run the launchers.

## Usage
Create a Python environment with `mujoco` installed. Then run `python launch/run_sim.py` for the wave-only motion test or `python launch/run_hybrid.py` for the hybrid mooring loop.
