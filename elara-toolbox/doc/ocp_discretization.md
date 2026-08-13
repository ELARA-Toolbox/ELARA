# Choosing an OCP Discretization

The discretization determines how the continuous-time optimal-control problem is transcribed into the finite-dimensional NLP.
More precisely, it determines how the system dynamics, running costs, and constraints are discretized in time.
Hence, the discretization influences which state variables appear in the NLP and how the dynamics and running costs are enforced between time nodes.

The selected method is determined by the discretization object assigned to `problem.discretization`.
It must be assigned and configured before `problem.initSolver` is called.

## Available Methods

ELARA provides one variational transcription and three ODE-based alternatives. Their main differences are the node variables, the discrete dynamics constraints, and the formal integration order.

| Discretization | Node variable | Dynamics constraint | Order |
|---|---|---|---:|
| `elara.ocp.DiscretizationVI` | Configuration `q_k` | Forced discrete Euler-Lagrange equations | 2 with trapezoidal dissipation |
| `elara.ocp.DiscretizationRK("RK2")` | State `[q_k; qDot_k]` | Explicit two-stage Heun step | 2 |
| `elara.ocp.DiscretizationRK("RK4")` | State `[q_k; qDot_k]` | Classical explicit four-stage RK step | 4 |
| `elara.ocp.DiscretizationImplicitMidpoint` | State `[q_k; qDot_k]` | Implicit midpoint step | 2 |

```matlab
problem.discretization = elara.ocp.DiscretizationVI;

% Alternatives
problem.discretization = elara.ocp.DiscretizationRK("RK2");
problem.discretization = elara.ocp.DiscretizationRK("RK4");
problem.discretization = elara.ocp.DiscretizationImplicitMidpoint;
```

Changing the discretization changes the NLP structure. `initSolver` must therefore be called again, and an initial guess with the corresponding dimensions must be provided.

## Variational/DMOC Discretization

The variational formulation stays closest to the underlying mechanical principles and uses configurations as its only node states. `DiscretizationVI` discretizes the Lagrange-d'Alembert principle before the equations of motion are formed. Consecutive configurations are the discrete state, and the forced discrete Euler-Lagrange equations become equality constraints in the NLP. In simplified notation, these constraints have the form

$$
D_2L_d(q_{k-1},q_k)
+D_1L_d(q_k,q_{k+1})
+F_k=0,
$$

where $L_d$ approximates the mechanical action over one interval, $D_1$ and $D_2$ denote derivatives with respect to its two configurations, and $F_k$ collects discrete actuation and dissipation. Initial and optional final velocity conditions are imposed through the corresponding discrete momentum relations.

This formulation preserves the Lie-group configuration geometry and the variational structure of the mechanical model. In practical terms, rotations remain valid rotations throughout the transcription, while the discrete dynamics retain important properties of the original mechanical system. Approximately half as many state variables are used as in an ODE transcription because velocities are not independent node variables. It is the default and usually provides a suitable starting point for rigid-flexible systems.

The dissipation quadrature, which determines how the damping contribution is approximated over each interval, is configurable:

```matlab
d = elara.ocp.DiscretizationVI;
d.aTrapez = 0.5;  % trapezoidal dissipation, always second order
problem.discretization = d;
```

`aTrapez = 0` uses the simpler rectangle-rule dissipation term, whereas `aTrapez = 0.5` uses the full trapezoidal term.
With `aTrapez = 0`, the discretization is first order when dissipation is present and remains second order for conservative systems.
With `aTrapez = 0.5`, it is always second order.
Note that this property corresponds to the `useFirstOrderDissipation` property of the variational integrator for numerical simulations.

The running cost is integrated with the trapezoidal rule.

For the derivation, see Chapter 5 of the dissertation and the variational-model references under [Further Reading](index.md#further-reading).

## Runge-Kutta Discretizations

Runge-Kutta methods provide a familiar alternative for transcribing the system in conventional first-order ODE form. `DiscretizationRK` applies one explicit Runge-Kutta (RK) step to the state equation in each NLP interval. Each step defines a map

$$
x_{k+1}=\Phi_h(x_k,u_k,u_{k+1}),
$$

where $\Phi_h$ denotes the selected RK approximation over a time interval of length $h$. `RK2` is cheaper per interval; `RK4` is the constructor default and provides a higher integration order for sufficiently smooth problems.

For direct node-based controls, stage inputs are interpolated linearly between $u_k$ and $u_{k+1}$. With B-spline controls, the spline is evaluated at the actual RK stage times. The running-cost quadrature uses the corresponding RK weights.

ODE initial guesses must contain both coordinates and velocities:

```matlab
xInit = [qInit; qDotInit];
[xSol, uSol] = problem.solve(xInit, uInit);
```

## Implicit-Midpoint Discretization

The implicit-midpoint method provides a second-order ODE transcription with one midpoint evaluation per interval. `DiscretizationImplicitMidpoint` enforces the first-order dynamics according to

$$
x_{k+1}=x_k+h f \left(
\frac{x_k+x_{k+1}}{2},u_{k+\frac12}
\right).
$$

Both state and direct input midpoints are averages of their endpoint values; a B-spline control is evaluated at the midpoint. The running cost uses midpoint quadrature.

Although the step relation is implicit, it is imposed directly as part of the NLP and does not require configuring a separate time-step solver.

## Selecting a Step Size

The temporal resolution controls the usual compromise between transcription accuracy and NLP size. All methods use the uniform step `problem.h`, and the dependent properties satisfy

$$
N=\mathrm{round} \left(\frac{t_{\mathrm{End}}}{h}\right),
\qquad
\boldsymbol{t}=\begin{bmatrix}0&h&\cdots&Nh\end{bmatrix}^{\mathsf T},
$$

These values can be read directly with:

```matlab
nSteps = problem.nSteps;
tout = problem.tout;
```

`nSteps` and `tout` are read-only dependent properties; only `tEnd` and `h` are set. A step that divides the horizon exactly is recommended.

A smaller `h` usually improves transcription accuracy but increases the number of variables and constraints. When accuracy matters, solutions can be compared on successively refined grids. A higher-order method does not guarantee a better solution on a coarse grid when bounds or nonsmooth active constraints dominate.

## Controls and Bounds

The selected state discretization can be combined with either direct or spline-based control variables. With `useSplineInputs = false`, every method uses control variables at the state nodes. With splines, the optimization variables are control points, and ELARA evaluates the resulting trajectory at the times required by the selected dynamics and cost functions.

Input limits for spline controls are enforced at the OCP nodes. A spline can overshoot between nodes, including at RK stage times. When strict continuous-time limits are required, increase the node density, inspect the reconstructed control, or add an application-specific margin.

See also [Solving Optimal-Control Problems](optimal_control.md) and the three optimal-control scripts in the `examples` folder.
