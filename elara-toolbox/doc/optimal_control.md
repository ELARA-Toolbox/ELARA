# Solving Optimal-Control Problems

ELARA can be used to efficiently solve optimal control and trajectory tracking problems for rigid-flexible systems and soft robots.
The full optimal control workflow is handled with the `elara.ocp.Problem` class, which defines a fixed-horizon trajectory-optimization problem for an ELARA system. It brings together:

* the discretized dynamics and boundary conditions,
* the running and final costs,
* coordinate and input bounds, and
* optional workspace constraints.

These components are transcribed into a nonlinear program (NLP), from which the configured solver is constructed through CasADi.

To use the optimal control functionalities, CasADi must be installed and on the MATLAB path. Its availability can be checked with `elara.setup`.

## Problem Formulation

ELARA uses direct transcription: the continuous trajectory is represented by a finite number of values, allowing the complete control problem to be handled by a standard nonlinear-programming solver. Configurations or states are optimization variables at a set of time nodes, while controls are represented either by node variables or by B-spline control points. Schematically, the resulting problem is

$$
\begin{aligned}
\underset{z}{\operatorname{minimize}}\quad
&J(z)=\sum_{k=0}^{N-1}\,\ell(x_k,u_k,w)+\Phi(x_N,u_N,w),\\
\text{subject to}\quad
&c(z)=0,\qquad g(z)\leq 0,
\end{aligned}
$$

where $z$ collects all optimization variables, $\ell$ and $\Phi$ are the running and final costs, and $w$ are the numerical quadrature weights that define the costs. The constraints contain the discretized dynamics, boundaries, and bounds.

The variational discretization uses only configurations $q_k$ as state variables and imposes discrete Euler-Lagrange equations. ODE discretizations instead use

$$
x_k=\begin{bmatrix}q_k\\\dot q_k\end{bmatrix}.
$$

The practical differences are described under [Choosing an OCP Discretization](ocp_discretization.md).

Detailed derivations are given in Chapter 5 of the dissertation listed under [Further Reading](index.md#further-reading).

## Minimal Workflow

This example moves a two-link planar robot between fixed configurations using a variational transcription:

```matlab
exampleFolder = fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems");
addpath(exampleFolder)

links = systemDef_planarNLinkPendulum("nLinks", 2, "d", 0);
problem = elara.ocp.Problem(links);

n = problem.systemNum.nDoF;
m = problem.systemNum.nInputs;

problem.tEnd = 1;
problem.h = 1e-2;
problem.q0 = [pi/2; 0];
problem.qDot0 = zeros(n,1);
problem.qF = [-pi/2; 0];
problem.qDotF = zeros(n,1);

problem.uMin = -25*ones(m,1);
problem.uMax =  25*ones(m,1);
problem.qMin = [-inf; -4];
problem.qMax = [ inf; 0.1];

problem.runningCostWeights = [0.5; 0; 0; 0; 0];
problem.runningCostActive = logical(problem.runningCostWeights);
problem.finalCostWeights = zeros(3,1);
problem.finalCostActive = logical(problem.finalCostWeights);

problem.discretization = elara.ocp.DiscretizationVI;

qInit = repmat(problem.q0, 1, problem.nSteps + 1);
uInit = zeros(m, problem.nSteps + 1);

problem = problem.initSolver;
[qSol, uSol, uDecision, sol, stats] = ...
    problem.solve(qInit, uInit);
```

The `Problem` class has value semantics, so the result of `initSolver` needs to be retained. The NLP graph is built during this call; consequently, the system, horizon, discretization, active cost terms, and constraints must be configured beforehand.

## Configuring the Horizon and Boundaries

Every transcription is defined on a uniform temporal mesh and may be supplemented with initial, final, and box constraints. The horizon settings `tEnd` and `h` determine the number and location of the transcription nodes:

$$
N=\operatorname{round}\!\left(\frac{t_{\mathrm{End}}}{h}\right),
\qquad t_k=kh,\quad k=0,\ldots,N.
$$

Ideally, `h` divides `tEnd`; otherwise the final node can differ slightly from the requested horizon.

Further properties are:
| Property | Meaning |
|---|---|
| `q0`, `qDot0` | Initial configuration and velocity. |
| `qF` | Optional final-configuration equality constraint. |
| `qDotF` | Optional final-velocity constraint; an empty value disables it. |
| `u0` | Optional fixed initial input for direct node-based control variables. |
| `qMin`, `qMax` | Coordinate bounds applied at all nodes. |
| `uMin`, `uMax` | Input bounds applied over the trajectory. |
| `x_TCP_F` | Desired final TCP position. |
| `addTCPFinalTimeConstraint` | Whether `x_TCP_F` is enforced as a final position equality rather than used only as a cost target. |

A final TCP orientation constraint is not currently implemented. The TCP is defined on one link before the problem is constructed; when no TCP is defined, TCP-related routines use the final system frame where supported.

`problem.simPars` is the `elara.SimulationParameters` object used by the dynamics. Gravity is configured through `problem.simPars.g`; sampled inputs and external-wrench profiles are not transcribed into the current optimal-control model.

## Configuring the Cost

Running and final costs use a weight vector and a separate logical activation vector. Both should be set explicitly, since the default activation masks are not inferred from the weights.
The activation vector defines which costs are included in the NLP during assembly with the `initSolver` call; it is separated from the actual weight values so one can easily change the weights after assembly.

| Running cost index | Quantity integrated over time |
|---:|---|
| `1` | Squared input norm `u` |
| `2` | Squared input-rate norm `u_dot` |
| `3` | Squared input-acceleration norm `u_ddot` |
| `4` | Squared coordinate-acceleration norm `q_ddot` |
| `5` | Squared TCP trajectory error |

| Final cost index | Quantity at `tEnd` |
|---:|---|
| `1` | Squared input norm |
| `2` | Squared coordinate norm |
| `3` | Squared TCP position error relative to `x_TCP_F` |

For example, they can be defined with:
```matlab
problem.runningCostWeights = [1e-3; 1e-2; 0; 1e-4; 0];
problem.runningCostActive  = logical(problem.runningCostWeights);

problem.finalCostWeights = [0; 0; 1e5];
problem.finalCostActive  = logical(problem.finalCostWeights);
```

If running TCP tracking (index 5) is active, `x_TCP_traj` needs to contain a 3-by-`nSteps+1` reference array. A point-to-point reference can be generated with:

```matlab
problem.x_TCP_F = [0.55; 0.30; 0.05];
problem.x_TCP_traj = ...
    elara.ocp.computeLinearReferenceTCPTrajectory(problem, ...
        "tPreAct", 0.1, "tPostAct", 0.1);
```

`finiteDifferenceOrder`, either `2` or `4`, controls derivatives used in the cost when they are obtained from node values.

## Parameterizing the Controls

The control trajectory can either be represented directly at every node or through a smaller set of smooth B-spline control points. With direct parameterization, the input at every time node is an NLP decision variable:

```matlab
problem.useSplineInputs = false;
uInit = zeros(problem.systemNum.nInputs, problem.nSteps + 1);
```

For long horizons, a B-spline can reduce the number of control decision variables and impose a smooth parameterization:

```matlab
problem.useSplineInputs = true;
problem.nInputSplinePoints = 30;
problem.inputSplineOrder = 3;

B = problem.getInputSplineBasisMatrix;
uNodeGuess = zeros(problem.systemNum.nInputs, ...
    problem.nSteps + 1);
uDecisionInit = (B \ uNodeGuess.').';
```

When splines are enabled, `solve` expects an `nInputs`-by-`nInputSplinePoints` matrix of control points. It returns both the evaluated node trajectory `uSol` and the optimized control points `uDecision`. The discretization evaluates the spline at any intermediate dynamics stages that it needs.

Spline input bounds are enforced at the OCP nodes, not continuously between them. When limits are strict, the reconstructed input should therefore be checked for overshoot.

## Choosing a Discretization and NLP Solver

Once the horizon, costs, and control parameterization have been defined, a transcription method is selected for the dynamics. The available choices are configured as follows:

```matlab
% Variational/DMOC transcription
problem.discretization = elara.ocp.DiscretizationVI;

% Or an ODE transcription
problem.discretization = elara.ocp.DiscretizationRK("RK4");
% problem.discretization = elara.ocp.DiscretizationImplicitMidpoint;
```

The CasADi graph and the selected NLP solver are then constructed with:

```matlab
problem.solver = "ipopt";  % Default; e.g., "sqpmethod"
problem.nlpOptions.expand = false;
problem = problem.initSolver( ...
    "useCasadiStepFunctions", true, ...
    "showDebugPlots", false);
```

`solver` is the name of an NLP solver plugin available in the installed CasADi build. `nlpOptions` is passed directly to that plugin, so the available options depend on the selected solver. `showDebugPlots` displays the constraint-Jacobian sparsity pattern. `useCasadiStepFunctions` can reduce repeated graph construction for the variational dynamics; its best setting depends on the problem.

Ipopt is used by default. Other CasADi-supported solvers, such as `sqpmethod`, may be selected when available. After changing `solver` or `nlpOptions`, the NLP must be rebuilt with `problem = problem.initSolver`; its graph and solver memory can be released beforehand with `problem = problem.clearSolver`.

## Constructing Initial Guesses and Solving

The state initial guess depends on the discretization:

| Discretization type | `xInit` size |
|---|---|
| Variational | `nDoF`-by-`nSteps+1`, containing `q` |
| ODE | `2*nDoF`-by-`nSteps+1`, containing `[q; qDot]` |

Control initial guesses contain node values for direct parameterization and B-spline control points for spline parameterization.

For difficult point-to-point problems, `elara.ocp.computeInitialGuessInvDyn` constructs a smooth configuration trajectory and computes approximate controls with continuous or discrete inverse dynamics:

```matlab
[qInit, qDotInit, uNodeInit, simInitialGuess] = ...
    elara.ocp.computeInitialGuessInvDyn(problem, ...
        "invDynMethod", "ODE", ...
        "createDebugPlots", false);
```

The helper can target `qF`, a final TCP position, or configured TCP waypoints. Its optional higher-rate generation and forward-simulation settings are illustrated in the examples and described in the source comments.

The returned node input can be converted to the active control parameterization as follows:

```matlab
if problem.useSplineInputs
    B = problem.getInputSplineBasisMatrix;
    uDecisionInit = (B \ uNodeInit.').';
else
    uDecisionInit = uNodeInit;
end
```

For an ODE transcription, the coordinate and velocity guesses are stacked before solving:

```matlab
xInit = [qInit; qDotInit];
[xSol, uSol, uDecision, sol, stats] = ...
    problem.solve(xInit, uDecisionInit);
n = problem.systemNum.nDoF;
qSol = xSol(1:n,:);
qDotSol = xSol(n+1:end,:);
```

### Solver Warm-Start
A related problem with compatible NLP variable and constraint dimensions can be warm-started from a previous CasADi solution containing `x`, `lam_x`, and `lam_g`:

```matlab
[xSol, uSol, uDecision] = problem.solve( ...
    xInit, uDecisionInit, "solWarmStart", sol);
```

## Workspace Constraints

In the optimal control framework, it is possible to incorporate workspace constraints into the OCP; for example, obstacles that must be avoided by the system.
Such geometric point constraints are added when the system frames or TCP need to remain inside or outside selected regions. These regions are represented by axis-aligned or rotated boxes in `elara.Workspace`:

* Type `0` defines an obstacle whose interior must be avoided.
* Type `1` defines an allowed workspace interior.

The workspace is configured before `initSolver`, because its constraints are built into the NLP graph. After a workspace change, the solver needs to be cleared and rebuilt with `problem = problem.clearSolver` followed by `problem = problem.initSolver`.

```matlab
problem.workspace = elara.Workspace;
problem.workspace = problem.workspace.addBoxSideLengths( ...
    [0.35; 0.10; 1.00], ...   % center
    zeros(1,3), ...           % Euler angles
    [0.25, 0.30, 0.30], ...   % side lengths
    0);                       % obstacle
```

The constraints use smooth signed-distance approximations, which provide the gradient-based solver with a differentiable measure of whether a point lies inside or outside a box. They are imposed on every system-frame origin, plus the TCP when one is defined, at the transcription nodes. They do not represent the volume between frame origins. When these pointwise checks are insufficient, a finer beam discretization or a suitable safety margin may be used. Workspace constraints can substantially increase NLP construction and convergence time.

The boxes can be inspected before solving with `problem.workspace.visualize`.

## Diagnosing and Post-Processing a Solution

Initial guesses and optimized trajectories can be checked through the same residual and objective functions used to construct the NLP. The following methods are available before and after `solve`:

```matlab
problem.plotConstraintResiduals(xInit, uDecisionInit);
[J, runningComponents, finalComponents] = ...
    problem.evaluateObjectiveComponents(xInit, uDecisionInit);

elara.ocp.plot.coordinatesInputs(problem, qSol, uSol, ...
    "plotDerivatives", true, ...
    "q_dot", qDotSol);
```

For TCP tracking, `elara.ocp.plot.TCPTrajectory(problem, qSol)` is available. Constraint residuals and objective components can be evaluated only after `initSolver` because they use functions generated with the NLP.

A configuration trajectory can be converted into standard simulation results so that the plotting and animation tools can be reused:

```matlab
sim = problem.getSimulationObject;
sim.Name = "Optimization";
sim.results = elara.SimulationResults.fromStateTrajectory( ...
    problem.systemNum, problem.tout, qSol, qDotSol);

sim.drawSnapshots;
sim.animateSimResults;
```

For a variational solution, the first output of `solve` is already `qSol`. When the velocity argument is omitted, it is estimated by `fromStateTrajectory` using finite differences.

> **Note regarding model consistency:** `problem.links`, `problem.systemNum`, and `problem.systemSym` are assembled as separate values and are not synchronized after construction. The `Problem` therefore needs to be reconstructed after link or system parameters have been changed.

## Related Examples

The supplied examples cover increasingly detailed optimal-control workflows:

* `optimal_control_planar_robot.m` -- final configuration, coordinate bounds, VI and RK4 comparison.
* `optimal_control_rigid_robot.m` -- TCP tracking, obstacle avoidance, B-spline inputs, VI and RK2 comparison.
* `optimal_control_continuum_manipulator.m` -- tendon actuation, inverse-dynamics initial guess, and VI transcription.

They can be used as templates for combining the settings introduced above.

See also [Defining Multibody Systems](system_definition.md), [Choosing an OCP Discretization](ocp_discretization.md), and [Visualizing Systems and Results](visualization.md).
