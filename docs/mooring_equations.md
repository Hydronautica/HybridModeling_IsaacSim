# Mooring model notes

The example mooring model uses a linear spring-damper wrench:

\[ F = -K x - D \dot{x} \]

For more realism, replace `compute_mooring_force` with a catenary solver or frequency-domain hydrodynamics.
