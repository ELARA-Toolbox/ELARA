# Running Numerical Simulations

Dynamic forward simulations are performed using the `elara.Simulation` class, which serves as the central object for a simulation workflow. It keeps three groups of data together:

* the link definition and assembled numeric system,
* the simulation parameters and integration method, and
* the resulting trajectory.

This makes the same object available for setup, execution, and post-processing.

## Basic Workflow

The complete workflow from a link definition to an integrated trajectory is collected in the `Simulation` object. A compact setup is shown below:

```matlab
systemFolder = fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems");
addpath(systemFolder)

links = systemDef_rigid_robot;
sim = elara.Simulation(links, "displayInfo", false, ...
    "Name", "Rigid robot");

sim.parameters.tEnd = 10;
sim.parameters.q0 = sim.system.setJointAngles( ...
    deg2rad([30; -30; -60]));
sim.parameters.qDot0 = zeros(sim.system.nDoF, 1);
sim.parameters.uConst = zeros(sim.system.nInputs, 1);

sim.integrator = elara.integration.VIBroyden;
sim.integrator.h = 2^-8;

sim = sim.simulateSystem;
```

The constructor assembles `sim.system` as an `elara.SystemNum`. With `displayInfo = true` (the default), link, frame, and input data are printed and the link and frame graphs are opened. This output can be disabled for scripts and batch runs.

The default integrator is already `elara.integration.VIBroyden`. The available alternatives and their settings are summarized under [Choosing an Integration Method](integration.md).

## Simulation Parameters

Initial conditions, prescribed inputs, gravity, and the simulation horizon are kept together as simulation parameters. They are stored in `sim.parameters`, an `elara.SimulationParameters` object.

| Property | Size | Description |
|---|---:|---|
| `q0` | `nDoF`-by-1 | Initial generalized coordinates. |
| `qDot0` | `nDoF`-by-1 | Initial generalized velocities. |
| `tEnd` | scalar | Simulation end time. |
| `g` | scalar | Gravitational acceleration; default `9.81`. |
| `uConst` | `nInputs`-by-1 | Input held constant over the simulation. |
| `uSampleValues` | `nInputs`-by-`nSampleTimes` | Samples of a time-varying input. |
| `uSampleTimes` | `nSampleTimes`-by-1 | Times corresponding to `uSampleValues`. |
| `externalWrench_b` | scalar object | Definitions of external wrenches in the body-fixed frame. |
| `externalWrench_s` | scalar object | Definitions of external wrenches in the spatial frame. |

The assembled dimensions are preferable to hard-coded sizes:

```matlab
sim.parameters.q0 = zeros(sim.system.nDoF, 1);
sim.parameters.qDot0 = zeros(sim.system.nDoF, 1);
sim.parameters.uConst = zeros(sim.system.nInputs, 1);
```

For systems with flexible links or nonzero joint stiffness, `sim.system.qRef` is the stress-free coordinate vector. The helpers `setJointAngles` and `setLinkDeformations` set all unrelated entries to zero, so joint and beam initial conditions need to be combined with some care.

## Actuation and System Inputs

For systems with control inputs (such as actuated joints or tendon actuation for flexible links), it is possible to simulate scenarios with prescribed actuation.
It is possible to specify:

- constant input values $u_{\mathrm{const}}$ (that remain constant over the complete simulation), and
- time-varying trajectories $\widetilde{u}_{\mathrm{sample}}(t)$ in the form of sampled input values.

Constant and sampled inputs can be used together. Their combined value is

$$
u(t)=u_{\mathrm{const}}+\widetilde{u}_{\mathrm{sample}}(t),
$$

where $\widetilde{u}_{\mathrm{sample}}$ is linearly interpolated between the supplied samples and evaluates to zero outside the sample interval.

For example, a trajectory of control inputs can be specified as follows:
```matlab
tInput = linspace(0, sim.parameters.tEnd, 100).';
ramp = (1 - cos(pi*tInput/sim.parameters.tEnd))/2;
targetInput = zeros(sim.system.nInputs, 1);
targetInput(1) = 25;

sim.parameters.uSampleTimes = tInput;
sim.parameters.uSampleValues = targetInput .* ramp.';
```

Input indices of the global system inputs are assigned in the order of the supplied links when the system is assembled. Each actuated screw joint contributes one input, and a single tendon-actuated link contributes one input per tendon.
For a newly defined model, the resulting order can be checked through `sim.system.nInputs` and the constructor output.

## External Wrenches

Apart from the generalized actuator inputs, one can also simulate the effect of external forces (and moments) that directly act on the system's frames.
These wrenches are applied and stored in `elara.ExternalWrench` objects, each of which combines a moment $m$ and a force $f$ acting on a frame according to
$$
\boldsymbol{w}=
\begin{bmatrix}\boldsymbol{m}\\\boldsymbol{f}\end{bmatrix}
\in\mathbb{R}^6.
$$

The wrench array therefore has size `6`-by-`nFrames`, and one can define multiple external wrenches at different sets of frames or with different time evolutions, which are summed together.

The following example defines a force that is resolved in the spatial frame, points in the global z-direction, and acts on the last frame of the system:
```matlab
maximumWrench = zeros(6, sim.system.nFrames);
maximumWrench(:,end) = [0; 0; 0; 0; 0; 30];

sim.parameters.externalWrench_s = ...
    sim.parameters.externalWrench_s.addWrench( ...
        0, 1, 4, maximumWrench);
```
In this manner, one can add an arbitrary number of external wrench definitions.
The helper method `addWrench` takes the arguments `(startTime, endTime, interpolationType, maximumWrench)`.
The arguments `startTime, endTime, interpolationType` specify the time profile:

| Type | Profile between `startTime` and `endTime` |
|---:|---|
| `1` | Constant |
| `2` | Linear increase |
| `3` | Linear decrease |
| `4` | Smooth sinusoidal impulse |
| `5` | Smooth increase, then remain constant |
| `6` | Initially constant, then smooth decrease |

The object returned by `addWrench` needs to be stored. `externalWrench_b` is used when the components are expressed in each moving body-fixed frame, whereas `externalWrench_s` is used when they are expressed in the inertial frame. A spatial impulse is demonstrated in the Chiemsee example.

## Running and Inspecting the Simulation

After the model, parameters, and integrator have been configured, time integration is initiated through `simulateSystem`. This method returns a modified value object, which therefore needs to be retained:

```matlab
sim = sim.simulateSystem;
results = sim.results;
```

The most useful result fields are:

| Field | Size | Description |
|---|---:|---|
| `tout` | `nTimes`-by-1 | Solver output times. |
| `q`, `q_dot` | `nDoF`-by-`nTimes` | Generalized coordinates and velocities. |
| `g` | 4-by-4-by-`nFrames`-by-`nTimes` | Absolute $\mathrm{SE}(3)$ frame poses. |
| `eta` | 6-by-`nFrames`-by-`nTimes` | Body-fixed frame twists. |
| `solverIterations` | 1-by-`nTimes` | Iterations of an implicit solver, when available. |
| `solverResidual` | 1-by-`nTimes` | Final implicit residual, when available. |
| `solverExitFlag` | 1-by-`nTimes` | Implicit-solver status, when available. |
| `computationTime` | scalar | Measured integration time. |

Energy fields are populated only after energy post-processing:

```matlab
sim = sim.computeEnergies;
elara.plot.energies(sim.results);
```

This computes kinetic, gravitational potential, strain, and total energy. The gravitational potential is shifted such that $V_g(t_0)=0$; it is therefore not an absolute reference value. The value-object assignment is required.

## Plotting and Animating Results

Simulation results can be explored through a concise set of high-level plotting and visualization methods:

```matlab
sim.plotAll;
sim.drawSnapshots("nSnapShots", 12);
sim.animateSimResults("frameRate", 30);
```

More focused methods are available for:

* joint angles,
* frame positions and velocities,
* beam data, and
* solver statistics.

Geometry settings, movie export, and result conversion are described under [Visualizing Systems and Results](visualization.md).

## Common Problems

Most simulation issues can be traced to dimensions, reference configurations, input coverage, or value-object assignments. The following checks cover the common cases:

* **Dimension mismatch:** initial-condition and input sizes can be derived from `nDoF` and `nInputs` after assembly.
* **Unexpected flexible-link shape:** for precurved beams, $q=0$ differs from the stress-free vector `qRef`.
* **Input starts or ends abruptly:** samples may be added over the full simulation interval, or the sampled signal can intentionally be combined with `uConst`.
* **Implicit solver does not converge:** the variational-integrator step size should normally be reduced before residual tolerances are loosened.
* **Simulation method appears to have no effect:** the returned value may not have been retained; for example, `sim = sim.simulateSystem` is required.

Integrator-specific convergence problems are discussed in more detail under [Choosing an Integration Method](integration.md).

## Related Examples

The simulation examples demonstrate the main model and integration combinations:

* `simulation_rigid_robot.m` -- variational and ODE integration of a rigid robot.
* `simulation_rigid_flexible_system.m` -- stiff mixed system and `ode15s` comparison.
* `simulation_continuum_manipulator.m` -- sampled tendon inputs.
* `simulation_rigid_flexible_robot_PD_control.m` -- stiffness/damping-based joint control.
* `simulation_chiemsee_lecture_notes.m` -- precurvature, equilibrium, external wrench, energies, projections, and snapshots.

The individual scripts also show suitable parameter and visualization settings for each system type.

See also [Defining Multibody Systems](system_definition.md) and [Choosing an Integration Method](integration.md).
