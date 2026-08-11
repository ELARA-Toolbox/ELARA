# Visualizing Systems and Results

ELARA comes with extensive tools for visualizing systems, simulation results, and plotting simulation data.

ELARA separates visualizations of specific system configurations from visualizations or plots of simulation results. High-level methods on `elara.Simulation` cover the common workflow; classes in `elara.visualization`, `elara.plot`, and `elara.ocp.plot` provide finer control.

## Configuring Link Geometry

To obtain insightful system visualizations, the corresponding properties of the system's links must be set; these are purely for visualization and do not affect the dynamics.
These properties are directly set for the individual links:

* A rigid link is drawn as a simple box, which is constructed according to the transformation `g_bbox` and the positive/negative extents in `bBoxSize`.
* A flexible link uses `beamParameters.height` and `beamParameters.width` for its cross-sections.
* Joint markers, body and beam frames, tendon paths, and the TCP are generated from the model definition.

These values are best set while the links are being defined, so that early configuration checks already show meaningful geometry.

## Visualizing a System Configuration

It is straightforward to visualize the system in the reference configuration (where $q=0$), or at a specific configuration given by a generalized coordinate vector $q$:

```matlab
% Visualize reference configuration (q = 0)
simulation.visualizeSystemRefConf;

% Visualize a specific configuration, e.g., the stress-free configuration
q = simulation.system.qRef;
[fig, vis] = simulation.visualizeSystemConfig(q, ...
    "figureName", "Stress-free configuration", ...
    "ShowInertialFrame", true, ...
    "linkColorMap", @lines);
```

This can be used to check the model geometry and reference shape.
`visualizeSystemRefConf` uses $q=0$. For a precurved beam whose allowed reference deformation is nonzero, the stress-free shape is obtained by displaying `system.qRef` instead.

The returned object `vis` is an `elara.visualization.SystemVisualization` object. It can update an existing drawing or add a planar projection:

```matlab
g = simulation.system.computeFwdKin(q);
vis.updateConfiguration(g);
vis.plotProjection("xz", fig.CurrentAxes.YLim(2));
```

For direct control of visible elements, the graphics object can be initialized explicitly:

```matlab
elara.visualization.initializeAxes("Name", "Model");
g = simulation.system.computeFwdKin(q);
vis = elara.visualization.SystemVisualization( ...
    simulation.system, simulation.links, g, ...
    "showInertialFrame", true, ...
    "showLinkFrames", false, ...
    "showBeamFrames", false, ...
    "showJoints", true, ...
    "showTendons", true);
```

A target pose can be marked with `elara.visualization.CoordinateFrame(gTarget)`. `initializeAxes("createFigure", false)` applies ELARA's grid, equal-axis, view, and labels to the current axes, which is useful for tiled layouts and overlays.

## Drawing Snapshots and Animations

After a simulation or OCP trajectory has been stored in `simulation.results`, it can be easily visualized and animated:

```matlab
simulation.drawSnapshots( ...
    "nSnapShots", 15, ...
    "includeColorbar", true, ...
    "snapShotColormap", @winter);

simulation.animateSimResults( ...
    "frameRate", 30, ...
    "saveMovie", false);
```

Both methods interpolate the generalized coordinates to evenly spaced display times and recompute forward kinematics. This changes only the visualization sampling, not the numerical result.

To combine an animation with a target or workspace, the figure can be prepared beforehand:

```matlab
fig = elara.visualization.initializeAxes( ...
    "Name", "Optimized motion");
elara.visualization.CoordinateFrame(gTarget);
workspace.visualize("createFigure", false);

simulation.animateSimResults("figure", fig);
```

Movie export is enabled with `saveMovie = true` and a full `fileName`. MATLAB `VideoWriter` is used with the MPEG-4 profile, and the figure size should remain fixed during capture. MPEG-4 support depends on the MATLAB platform.

## Plotting Simulation Data

Once a trajectory has been simulated, `plotAll` provides a quick overview of the available coordinate, frame, beam, and solver data:

```matlab
simulation.plotAll;
```

It does not calculate or plot energies, which require separate post-processing.

| Method | Content |
|---|---|
| `plotJointAngles` | Revolute-joint angles and angular velocities, in degrees. |
| `plotFramePositions` | Rigid-link frames and the first/last nodes of flexible links. |
| `plotFrameVelocities` | Body-fixed angular and translational frame velocities. |
| `plotBeamData` | Beam-node configurations and velocities and selected segment strains. |
| `plotSolverStats` | Integrator-specific step or implicit-solver statistics. |

Energy plots require a separate computation:

```matlab
simulation = simulation.computeEnergies;
elara.plot.energies(simulation.results, ...
    "nameString", simulation.Name);
```

For large beams, `plotBeamData` samples a limited set of nodes and segments to keep the figure set manageable. Custom plots can be created from `simulation.results` and `system.getLinkDeformations`.

## Plotting Optimal-Control Data

For a variational transcription, the configuration trajectory returned by `solve` can be plotted directly:

```matlab
[qSol, uSol, uDecision] = problem.solve(qInit, uInit);

elara.ocp.plot.coordinatesInputs(problem, qSol, uSol, ...
    "plotDerivatives", true, ...
    "FDOrder", 4);

problem.plotConstraintResiduals(qSol, uDecision);
elara.ocp.plot.TCPTrajectory(problem, qSol);
```

The optimized control decision variables `uDecision` are used for constraint residuals. With B-spline controls these are control points, whereas `uSol` contains values evaluated at the OCP nodes. `TCPTrajectory` is intended for problems with a configured reference `x_TCP_traj`.

For an ODE transcription, the returned state is split before plotting:

```matlab
[xSol, uSol, uDecision] = problem.solve(xInit, uInit);
n = problem.systemNum.nDoF;
qSol = xSol(1:n,:);
qDotSol = xSol(n+1:end,:);

elara.ocp.plot.coordinatesInputs(problem, qSol, uSol, ...
    "q_dot", qDotSol);
problem.plotConstraintResiduals(xSol, uDecision);
```

An OCP configuration trajectory can be converted so that all simulation visualizations can be reused:

```matlab
simulation = problem.getSimulationObject;
simulation.Name = "Optimization";
simulation.results = elara.SimulationResults.fromStateTrajectory( ...
    problem.systemNum, problem.tout, qSol);

simulation.plotAll;
simulation.drawSnapshots;
simulation.animateSimResults;
```

If velocities are omitted, `fromStateTrajectory` estimates them using second- or fourth-order finite differences. For an ODE transcription, `qDotSol` can be passed as the fourth argument in the call above.

## Workspace Graphics

Workspace geometry is often useful as a visual reference when an optimized motion is inspected. `problem.workspace.visualize` draws the configured interiors and obstacles as translucent polytopes and can either create a new figure or overlay the current axes:

```matlab
problem.workspace.visualize( ...
    "figureName", "Workspace", ...
    "createFigure", true);
```

The drawing is a diagnostic representation. OCP collision constraints operate on frame origins and the TCP, not on the rendered volume of each link.

See also [Running Numerical Simulations](simulation.md), [Solving Optimal-Control Problems](optimal_control.md), and the plotting sections of the supplied examples.
