# Hybrid Floating Body and Mooring Control Sandbox

This repository provides a skeleton Isaac Sim project that demonstrates how to couple a numerically evaluated mooring model with a physically simulated floating body and a Franka arm applying equivalent forces. The layout mirrors research-style Isaac Sim extensions so the system can evolve into a full hybrid experimental setup.

## Repository layout
```
hybrid_floater_system/
├── extensions/
│   ├── hybrid.mooring/        # Hybrid controller, Franka control, UI helpers
│   └── floating_body/         # Floater asset, wave and hydrodynamic helpers
├── launch/                    # Entrypoints for standalone or hybrid runs
└── docs/                      # High-level design notes and diagrams
```

## Quick start (conceptual)
1. Launch `run_hybrid.py` to spawn the floater and Franka arm inside Isaac Sim 5.0.
2. The `HybridController` samples the floater pose and velocity, calls the numerical mooring model, maps the resulting wrench to the Franka end-effector, and feeds torques to the articulation controller.
3. Use `drag_grab_enable.py` to toggle PhysX grab-and-drag during debugging.

The scripts are intentionally lightweight: they are meant as templates for integrating real hydrodynamics, mooring solvers, and control laws.

## Usage
Run the launchers with the Isaac Sim 5.0 Python interpreter so that modules such as `omni.isaac.core` are discoverable. On Linux/macOS, use `python.sh`; on Windows, use `python.bat` from the Isaac Sim installation directory. Alternatively, set the environment variable `ISAACSIM_PYTHON_PATH` to `<isaac-sim-root>/python` before executing `launch/run_sim.py` or `launch/run_hybrid.py`. If Isaac Sim is not available on the Python path, the launchers will raise a clear error describing how to resolve the issue. The helper in `launch/isaac_path.py` also scans default Isaac Sim 5.x install directories (e.g., `~/.local/share/ov/pkg/isaac-sim-5.0.x/python`), including the corresponding `python/lib/python*/site-packages` subfolders, when environment variables are not set.
