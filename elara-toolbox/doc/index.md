# The ELARA Toolbox

ELARA (Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control) is a MATLAB toolbox for modeling, simulation, and optimal control of fixed-base rigid-flexible multibody systems. Its main capabilities include:

* open-tree robot kinematics on $\mathrm{SE}(3)$ and flexible links modeled as geometrically exact beams,
* simulation through standard ODE solvers or a Lie-group variational integrator, and
* CasADi-based direct optimal control.

These components share the same link-based model definition, allowing the same system model to be used for simulation and trajectory optimization with little additional setup.

The toolbox also supports modeling and optimal control of soft robots and continuum manipulators with tendon actuation.

The documentation is intentionally task-oriented. Complete argument lists and implementation details are available through `help`, in the well-commented class and function files, and in the scripts in the `examples` folder.

## Requirements and Setup

ELARA requires MATLAB R2025b or later. Signal Processing Toolbox is required by the public SO(3) and SE(3) exponential-map functions used in the core simulation workflow. Other components are needed only for the workflows in which they are used:

* CasADi is required for optimal control and other symbolic optimization workflows; version 3.7.2 is tested for this release.
* MATLAB Coder and a supported C++ compiler enable faster MEX implementations of performance-critical simulation functions.
* Symbolic Math Toolbox is used by the tendon-path derivative helper.
* Robotics System Toolbox is used for minimum-jerk TCP trajectories and rotated workspace boxes.
* Optimization Toolbox, Mapping Toolbox, and Robotics System Toolbox are used by the inverse-dynamics initial-guess helpers.

These other products are not required for the basic MATLAB simulation workflow.

After installation, the active configuration can be checked with:

```matlab
elara.setup
```

Missing MEX files or CasADi do not prevent use of the core MATLAB simulation implementation. The optional MEX functions can be built after configuring a C++ compiler:

```matlab
mex -setup C++
elara.build
elara.setup
```

The toolbox automatically selects an available MEX implementation without requiring changes to simulation code.

## First Simulation

In the standard workflow, the links are defined first. A simulation is then constructed and configured with initial conditions and an integration method. The example below uses one of the included system definitions:

```matlab
systemsFolder = fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems");
addpath(systemsFolder)

links = systemDef_rigid_robot;
sim = elara.Simulation(links, "displayInfo", false, "Name", "Rigid robot");

sim.parameters.tEnd = 5;
sim.parameters.q0 = deg2rad([30; -30; -60]);
sim.parameters.qDot0 = zeros(sim.system.nDoF, 1);
sim.parameters.uConst = zeros(sim.system.nInputs, 1);

sim.integrator = elara.integration.VIBroyden;
sim.integrator.h = 2^-8;

sim = sim.simulateSystem;
sim.plotJointAngles;
sim.animateSimResults;
```

`elara.Simulation` and most configuration classes have value semantics. Therefore, the returned object must be retained when it is modified by a method, as in `sim = sim.simulateSystem` and `sim = sim.computeEnergies`.

## Documentation Topics

The main workflows are described in the following topic pages:

* [Defining Multibody Systems](system_definition.md) explains rigid and flexible links, relative coordinates, beam deformation modes, actuation, and assembled system objects.
* [Running Numerical Simulations](simulation.md) covers initial conditions, inputs, external wrenches, result data, and post-processing.
* [Choosing an Integration Method](integration.md) compares the variational and ODE interfaces and summarizes their key settings.
* [Solving Optimal-Control Problems](optimal_control.md) describes the `elara.ocp.Problem` workflow, costs, constraints, initial guesses, and solution processing.
* [Choosing an OCP Discretization](ocp_discretization.md) compares variational, Runge-Kutta, and implicit-midpoint transcription.
* [Visualizing Systems and Results](visualization.md) lists the high-level plotting, snapshot, and animation tools.

Together, these pages follow the usual progression from a model definition to simulation or optimal control and, finally, result analysis.

## Conventions

ELARA uses SI units. Each homogeneous matrix combines a three-dimensional orientation and a position to represent a pose in $\mathrm{SE}(3)$. Six-dimensional twists, strains, screw axes, and wrenches store the rotational component first and the translational component second. For example,

$$
\boldsymbol{\eta} =
\begin{bmatrix}\boldsymbol{\omega}\\ \boldsymbol{v}\end{bmatrix},
\qquad
\boldsymbol{w} =
\begin{bmatrix}\boldsymbol{m}\\ \boldsymbol{f}\end{bmatrix},
$$

where $\boldsymbol{\omega}$ and $\boldsymbol{v}$ are angular and translational velocities, while $\boldsymbol{m}$ and $\boldsymbol{f}$ represent a moment and a force. Generalized coordinates and inputs are column vectors, while trajectory arrays store one time sample per column.

In ELARA, there are two representations of multibody systems:

* `elara.SystemNum` contains numeric arrays optimized for simulation and optional code generation.
* `elara.SystemSym` contains a CasADi-compatible representation used to construct optimization graphs.

Both representations have the same properties and methods but use different implementations optimized for their intended purposes.

## Examples

The `examples` folder contains complete, executable workflows for:

* rigid robots and rigid-flexible systems,
* a tendon-actuated continuum manipulator, and
* three optimal-control problems.

`simulation_rigid_robot.m` and `optimal_control_planar_robot.m` provide accessible starting points; the more specialized scripts can then be used as templates.

## Further Reading

The documentation gives only the mathematical context needed to use the software. Detailed derivations and numerical studies are available in:

* M. Herrmann, *Discrete Geometric Modeling and Optimal Control of Flexible Robot Manipulators*, PhD thesis, publication details forthcoming.
* M. Herrmann and P. Kotyczka, "Relative-kinematic formulation of geometrically exact beam dynamics based on Lie group variational integrators," *Computer Methods in Applied Mechanics and Engineering*, vol. 432, 117367, 2024. [doi:10.1016/j.cma.2024.117367](https://doi.org/10.1016/j.cma.2024.117367)
* M. Herrmann, L. Pfeiffer, and P. Kotyczka, "Discrete Geometric Modeling and Extended State Estimation of Continuum Robots," publication details forthcoming.
