# Hybrid control flow

The hybrid controller closes the loop between a numerically evaluated mooring model and a physically simulated floating body/robot arm pair.

1. **State estimation** – `floater_estimator.get_floater_state` reads the floater pose and twist.
2. **Mooring evaluation** – `mooring_model.compute_mooring_force` produces a restoring wrench from displacement and velocity.
3. **Force mapping** – `hybrid_controller.map_force_to_ee` resolves the floater-frame wrench into a target end-effector wrench.
4. **Control law** – `FrankaCartesianImpedance.step` blends impedance control with the feedforward wrench.
5. **Actuation** – `HybridController.step` submits joint torques through the Isaac Sim articulation API.
