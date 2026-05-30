<div align="center">
  <img src="doc/.assets/banner.png" alt="Elara: Efficient Lie-group Algorithms for Flexible Robot Analysis and Control" style="width:100%;height:auto;max-width:100%;"/>
</div>

[![View Elara Toolbox on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://de.mathworks.com/matlabcentral/fileexchange/) 
[![GitHub top language](https://img.shields.io/github/languages/top/ELARA-Toolbox/ELARA)](https://matlab.mathworks.com/)  
![GitHub Repo stars](https://img.shields.io/github/stars/ELARA-Toolbox/ELARA?style=social)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![MATLAB Version](https://img.shields.io/badge/MATLAB-R2025b%2B-blue)
![Work in Progress](https://img.shields.io/badge/status-WIP-orange)

# ELARA Simulation and Optimal Control Toolbox

ELARA, short for **Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control**, is a MATLAB toolbox for the simulation and optimal control of robotic (multibody) systems consisting of rigid and highly flexible links.
It combines Lie-group kinematics on SE(3), rigid multibody dynamics, geometrically exact beam models, structure-preserving variational integration, and CasADi-based direct optimal control.

## Features

- Supports general rigid-flexible multibody models with open kinematics consisting of rigid links, flexible beam links, and screw joints (i.e., revolute, prismatic, or screw joints)
- Actuation via actuated screw joints or tendon actuation for continuum manipulators
- Time integration with highly efficient Lie-group variational integrator or any standard ODE solver via general interface (including all MATLAB ODE solvers)
- Fully CasADi-compatible implementation, e.g., for optimal control, parameter identification, or system optimization
- Direct optimal control with various time discretizations, including variational, Runge-Kutta (2nd and 4th order), and implicit-midpoint discretizations
- Extensive plotting, visualization, and animation functionality
- MEX generation (optional, but recommended) for increased performance

## Requirements

- MATLAB `R2024b` and above

Optional dependencies:

- CasADi for MATLAB: required for optimal control (or other optimization-related tasks)
- MATLAB Coder and suitable C++ compiler for optional MEX generation

## Installation

- Recommended: Installation via the packaged toolbox file
```matlab
matlab.addons.install("elara-toolbox.mltbx")
```

Alternatively, use the source checkout directly from the repository root:

```matlab
addpath(genpath(fullfile(pwd, "elara-toolbox")))
```

For optimal control, install CasADi from <https://web.casadi.org/> and add it to the MATLAB path.

## Quick Start

Run one of the included examples:

```matlab
cd elara-toolbox/examples
simulation_rigid_robot
```

For optimal control:

```matlab
cd elara-toolbox/examples
optimal_control_planar_robot
```

The simulation examples demonstrate the standard workflow:

1. Define links with an example system function or custom `MBLinkDefinition` objects.
2. Create an `MBSimulation`.
3. Set `MBSim.simPars` for initial conditions, inputs, gravity, and final time.
4. Select a solver such as `MBSimIntegratorVarIntBroyden` or `MBSimIntegratorODEDirect`.
5. Run `simulateSystem`, then use plotting or animation helpers for post-processing.

The optimal-control examples demonstrate:

1. Creating an `OCPDefinition`.
2. Using `MBSystemSym(links)` for CasADi-compatible dynamics.
3. Selecting an OCP discretization such as `OCPIntegratorVI` or `OCPIntegratorRK`.
4. Initializing the NLP solver with `initSolver`.
5. Solving with `solve` and post-processing the resulting trajectory.

## Repository Layout

```text
elara-toolbox/
|-- build.m
|-- doc/
|-- elara/
|   |-- equations/
|   |-- initial_guess/
|   |-- integration/
|   |-- internal/
|   |-- math/
|   |-- optimal-control/
|   |-- plotting/
|   |-- system-definition/
|   `-- visualization/
`-- examples/
    `-- exampleSystems/

tests/
```

The most important subpackages are:

- `system-definition`: link, beam, frame, and multibody-system classes.
- `equations`: continuous and discrete equations of motion.
- `integration`: simulation classes, integrators, parameters, external wrenches, and result containers.
- `optimal-control`: CasADi OCP definition, discretizations, NLP construction, solver interface, B-spline helpers, and workspace utilities.
- `math`: SE(3)/SO(3), exponential and Cayley maps, adjoint maps, and CasADi-compatible variants.
- `plotting` and `visualization`: post-processing, 3D visualization, snapshots, and animation.

## Examples

Forward simulation:

- `simulation_rigid_robot.m`: three-link rigid robot.
- `simulation_rigid_flexible_system.m`: cantilever beam with attached rigid links.
- `simulation_rigid_flexible_robot_PD_control.m`: rigid-soft manipulator with stiffness/damping based pseudo-PD control.
- `simulation_continuum_manipulator.m`: tendon-actuated one-link continuum manipulator.

Optimal control:

- `optimal_control_planar_robot.m`: planar rigid-manipulator trajectory generation with ODE and variational discretizations.
- `optimal_control_continuum_manipulator.m`: tendon-actuated continuum-manipulator trajectory generation with B-spline input parameterization.
- `optimal_control_rigid_robot.m`: rigid-lab-robot trajectory generation. This script currently expects a lab-specific `systemDefLabRobotRigid` function; adapt it to an available system definition if that function is not on your MATLAB path.

Reusable model definitions are in `elara-toolbox/examples/exampleSystems`, including `systemDef_rigid_robot`, `systemDef_rigid_flexible_robot`, `systemDef_planarNLinkPendulum`, `systemDef_continuum_manipulator`, and `systemDef_cantilever_system`.

## Core Concepts

### System Definition

ELARA systems are built from arrays of `MBLinkDefinition` objects:

- `MBLinkDefinitionRigid` stores rigid-link mass, inertia, joint data, reference transformations, and optional visualization data.
- `MBLinkDefinitionFlexible` stores beam length, segment count, deformation-mode selection matrices, reference strains, beam parameters, optional attached masses, and optional cable actuation.
- `MBBeamParams` stores beam geometry, material data, stiffness, mass, inertia, and damping.

The assembled system object is either:

- `MBSystemNum`, used for numeric forward simulation, or
- `MBSystemSym`, used for CasADi-compatible symbolic dynamics in optimal control.

### Simulation

`MBSimulation` combines a link definition, assembled numeric system, simulation parameters, solver, and result object. Initial conditions, inputs, gravity, external wrenches, and final time are stored in `MBSimPars`.

Available simulation integrators include:

- `MBSimIntegratorVarIntBroyden`: Lie-group variational integrator with fixed step size `h`.
- `MBSimIntegratorODEDirect`: MATLAB `ode` object interface.
- `MBSimIntegratorODEDirectFunctionBased`: function-handle interface for solvers such as `ode45`.

After simulation, use `plotAll`, `computeEnergies`, `drawSnapshots`, and `animateSimResults` for post-processing.

### Optimal Control

`OCPDefinition` stores the time grid, boundary conditions, state and input bounds, cost weights, TCP targets or trajectories, optional B-spline input parameterization, and CasADi/Ipopt options.

Available OCP discretizations include:

- `OCPIntegratorVI`: variational/discrete Euler-Lagrange transcription.
- `OCPIntegratorRK("RK2")` and `OCPIntegratorRK("RK4")`: ODE transcriptions with Runge-Kutta integration.
- `OCPIntegratorImplicitMidpoint`: ODE transcription with implicit midpoint integration.

The helper `OCPComputeInitialGuess_InvDyn` can generate inverse-dynamics-based initial guesses for trajectory-optimization problems.

## Optional Code Generation

The toolbox runs with plain MATLAB `.m` files. To generate optional MEX files, run from the repository root:

```matlab
mex -setup C++
addpath(genpath(fullfile(pwd, "elara-toolbox")))
build
```

The simulation code automatically uses generated MEX functions when they are available on the MATLAB path and otherwise falls back to the MATLAB implementations.

## Testing

The repository includes script-based tests for exponential and Cayley map utilities:

```matlab
addpath(genpath("elara-toolbox"))
results = [
    runtests("tests/ExpMapTests.m")
    runtests("tests/CayleyMapTests.m")
];
disp(table(results))
```

The runner scripts in `tests/` refer to a local `pathdef_local` helper; if that helper is unavailable, use the direct `runtests` commands above.

## Limitations and Future Work

Planned or possible future extensions include recursive continuous and discrete dynamics algorithms, contact forces, closed-chain constraints, floating-base systems, URDF import, richer workspace/collision geometry, and broader automated tests for complete simulation and optimal-control workflows.

## License and Citation

ELARA is distributed under the MIT License. See [LICENSE](LICENSE).

If you use ELARA in academic work, please cite the toolbox release or repository version you used, including the version number or commit hash, and cite the associated method papers relevant to your use case.

Suggested software citation:

```bibtex
@software{herrmann_elara_2026,
  author  = {Herrmann, Maximilian},
  title   = {ELARA Simulation and Optimal Control Toolbox},
  year    = {2026},
  version = {0.1.0},
  license = {MIT}
}
```

Useful background references include:

- Murray, R. M., Li, Z., and Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
- Brudigam, J., Sosnowski, S., Manchester, Z., and Hirche, S. (2024). "Variational integrators and graph-based solvers for multibody dynamics in maximal coordinates." *Multibody System Dynamics*, 61(3), 381-414. DOI: `10.1007/s11044-023-09949-x`.
