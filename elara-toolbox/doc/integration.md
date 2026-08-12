# Choosing an Integration Method

ELARA supports several numerical integrators for forward simulation.
The selected integrator is stored in `simulation.integrator` and must be assigned and configured before `simulation.simulateSystem` is called.
By default, ELARA provides a fixed-step Lie-group variational integrator and two interfaces to standard ODE solvers.

## Available Integrators

The integration method can be selected according to the system stiffness, the preferred MATLAB solver interface, and whether structure-preserving time stepping is desired. The available classes are summarized below.

| Class | Formulation | Typical use |
|---|---|---|
| `elara.integration.VIBroyden` | Implicit, fixed-step variational integrator | Default; rigid-flexible and numerically stiff systems, structure-preserving simulation |
| `elara.integration.ODEDirect` | ODE solver for the system in first-order form through a MATLAB `ode` object | Current object-based MATLAB ODE workflow |
| `elara.integration.ODEDirectFunctionBased` | ODE solver for the system in first-order form through functions such as `ode45` or `ode15s` | Classic function-based MATLAB ODE workflow |

All integrators inherit two common settings:

* `showConsoleOutput` controls timing and solver information printed during a run; default `true`.
* `accurateTiming` repeats the integration with `timeit`; default `false`. Enable it only for benchmarking because it increases total runtime.

## Lie-Group Variational Integrator

With `elara.integration.VIBroyden`, an implementation of an implicit, fixed-step, first/second-order Lie-group variational integrator is available that can be highly efficient both for numerically non-stiff and stiff systems.
It provides a structure-preserving, numerically efficient alternative to conventional ODE solvers and is the ELARA default. As a variational integrator, it is based on a discrete Lagrange-d'Alembert principle: the mechanical balance laws are formulated directly in discrete time rather than by discretizing an ODE *a posteriori*. Configurations are advanced with Lie-group operations on $\mathrm{SE}(3)$. At each step, the next configuration is obtained from an implicit equation of the form

$$
F_{\mathrm{DEL}}\!\left(q_{k-1},q_k,q_{k+1},u_k\right)=0,
$$

which is solved with Broyden's good method, a quasi-Newton method that updates an approximation of the residual Jacobian instead of recomputing it at every iteration. In the conservative case, the integrator is second order and preserves the system's variational structure, which generally provides good long-term momentum and energy behavior.

The formulation operates on consecutive configurations rather than a doubled first-order state containing both position and velocity. Together with the relative-coordinate model, this keeps the implicit solve practical for many stiff flexible systems. An adequately small time step is nevertheless still required.

For the full derivation and numerical studies, see Chapters 3 and 4 and the 2024 paper listed under [Further Reading](index.md#further-reading).

### Configuration

For most applications, only the step size and the treatment of dissipation need to be selected initially. A representative configuration is:

```matlab
simulation.integrator = elara.integration.VIBroyden;
simulation.integrator.h = 2^-9;
simulation.integrator.tolerance = 1e-10;
simulation.integrator.useFirstOrderDissipation = false;

simulation = simulation.simulateSystem;
```

| Property | Typical Values | Description |
|---|---|---|
| `h` | `2^-8` | Fixed time step in seconds. |
| `tolerance` | `1e-7` to `1e-12` | Target norm of the implicit residual. |
| `toleranceLimit` | `1e-6` to `1e-10` | Largest residual accepted after the iteration limit. |
| `maxIterations` | `100` | Maximum Broyden iterations per step. |
| `JacobianIterationThreshold` | 3 to 5 | Recompute the Jacobian after a step whose iteration count exceeds this value. |
| `useFirstOrderDissipation` | `true` | Use the more robust but less accurate rectangle-rule approximation for dissipation. |

`useFirstOrderDissipation = false` uses a trapezoidal dissipation term and is second order even when damping is present. `true` is first order for dissipative systems but is often more robust and permits larger steps. The choice does not change the order for a conservative system.

The generated time grid is

$$
t_k=kh,\qquad
k=0,\ldots,\operatorname{round}\!\left(\frac{t_{\mathrm{End}}}{h}\right).
$$

When the exact endpoint matters, `h` should be chosen so that it divides `tEnd`.

### Implementation Details

- The primary solver setting is the time step `h`, which determines both the accuracy (integration error) and the computation time of the integrator.

   __Important:__ For numerically stiff systems, there is usually a _maximum time step_ $h_{max}$, which can not be exceeded for the solver to run successfully. For larger time steps $h > h_{max}$, the implicit solver will not converge, and the simulation will fail. 
   
   - Hence, if the variational integrator does not run for a specific system, the first thing to try is usually decreasing the time step, e.g., by factors of 2 or 10.
   
   - The maximum time step strongly depends on the dissipation in the system; with increased dissipation, larger time steps can be used.
   
- For systems with dissipation, the property `useFirstOrderDissipation` determines the integrator's order: For `useFirstOrderDissipation =  false`, it is second-order, for `useFirstOrderDissipation = true`, it is first-order.
  For conservative systems (without dissipation), this value has no effect, since it only affects the dissipation terms.
  
  For increased accuracy, one usually wants to use `useFirstOrderDissipation = false` for second-order accuracy. However, in cases where computational efficiency is the primary concern, `useFirstOrderDissipation = true` may be useful,
  since it usually results in a much higher maximum time step $h_{max}$.
  This means that much larger time steps can be used, which can significantly decrease computation time.
  
- The setting `tolerance` determines the residual threshold for the solution of the implicit system of equations.
  Adjusting this value is often advantageous for either increased numerical efficiency (since higher tolerances usually mean less solver iterations and thus less computation time), or increased robustness.
  In some cases, adjusting this value may allow the solver to run successfully when it failed for the default settings.
  However, it depends on the specific system, which values result in the highest stability.  
  
- For the highest performance and stability, one can additionally tune the parameters `toleranceLimit`, `maxIterations` and `JacobianIterationThreshold`.
  
  - If `toleranceLimit` is set to a larger value than `tolerance`, the integrator does not abort the simulation if the implicit solver does not converge to the default tolerance specified by `tolerance` within `maxIterations`, as long as the residual remains below `toleranceLimit`.
  
    This setting can be useful to successfully integrate systems with difficult numerical properties, but usually results in much larger run times since the solver executes a large number of additional iterations.
    
  - Correspondingly, `maxIterations` sets the maximum number of iterations of the implicit solver. If the residual does not converge to `toleranceLimit` within this number, the integration fails.
  
  - `JacobianIterationThreshold` defines how often the Jacobian of the implicit system of equations (i.e., the DEL Jacobian) is recomputed during integration.
   To increase performance, the Jacobian is not computed in each time step; it is only recomputed if the number of iterations in the last time step exceeds `JacobianIterationThreshold`.
   Otherwise, the Jacobian of the last time step is reused.
   Reusing the old Jacobian is a compromise between the computational cost of recomputing the exact Jacobian (which is expensive) and a few additional solver iterations caused by a slightly inaccarute Jacobian.
   There is usually an optimal value to achieve the highest perfomance, which often lies between 3 and 5.
   In some cases, this mechanism can impact the stability of the integrator, and using a smaller number (or even 0 to disable the feature) may help.


### Practical Quick-Start Guide for the Variational integrator

1. If the system is dissipative, decide whether the focus is on accuracy (low integration error), or on computational time. For accuracy, set `useFirstOrderDissipation` to `false`, otherwise to `true`.
   For conservative (non-dissipative) systems, skip this step, as this setting only influences the dissipation term.

2. Start with a fairly large time step `h`, e.g., the default value. If the integration does not succeed or the results show signs of instability (which may be the case for numerically stiff systems), decrease `h` until the integration completes successfully.
Otherwise, choose `h` according to the requirements on integration error and run time.

3. For highest performance, adjust `tolerance`, which may allow to run the simulation with a larger time step or a smaller number of iterations, resulting in faster run times.

4. For complex cases, you can further adjust `toleranceLimit` and the other settings mentioned above.

The result can be assessed through `simulation.plotSolverStats` or the fields `solverIterations`, `solverResidual`, and `solverExitFlag`. `VIBroyden` stores `NaN` in the final `q_dot` and `eta` samples because its discrete velocities belong to time intervals. `computeEnergies` uses finite-difference velocities by default; in custom post-processing, the final sample should be handled or reconstructed explicitly.

## Object-Based ODE Interface

MATLAB's object-based ODE interface keeps the solver choice and its options together in one object. ELARA exposes this workflow through `elara.integration.ODEDirect`, whose solver and tolerances are configured before the simulation starts:

```matlab
simulation.integrator = elara.integration.ODEDirect;
simulation.integrator.odeObject.Solver = "ode15s";
simulation.integrator.odeObject.AbsoluteTolerance = 1e-6;
simulation.integrator.odeObject.RelativeTolerance = 1e-6;

simulation = simulation.simulateSystem;
```

`ode45` is a useful starting point for nonstiff rigid systems. Flexible systems commonly require a stiff solver such as `ode15s`. Solver selection, tolerances, and other supported options follow the MATLAB `ode` object interface. Integration is performed over `[0, tEnd]`, and the adaptive points selected by the solver are stored. The current simulation wrapper does not expose a custom output grid.

## Function-Based ODE Interface

For existing code based on solver functions such as `ode45` and `ode15s`, the classic MATLAB interface remains available. It is exposed through `elara.integration.ODEDirectFunctionBased`:

```matlab
simulation.integrator = ...
    elara.integration.ODEDirectFunctionBased;
simulation.integrator.solverFunction = @ode15s;
simulation.integrator.solverOptions = odeset( ...
    "AbsTol", 1e-6, "RelTol", 1e-6);

simulation = simulation.simulateSystem;
```

The defaults are `@ode45` and `odeset()`.

### Derivative and Mass-Matrix Forms

Both ODE classes use the first-order state

$$
x=\begin{bmatrix}q\\\dot q\end{bmatrix}.
$$

They also inherit `useMassMatrixForm`, which is `false` by default. The two available representations are

$$
\dot x=f(t,x,u)
\qquad\text{or}\qquad
M(x)\dot x=f(t,x,u).
$$

In the derivative form, the acceleration is solved inside each model evaluation. In the mass-matrix form, $M$ and the right-hand side are passed separately to the selected MATLAB solver. A solver supporting a state-dependent mass matrix is required for the latter form.

## MEX Acceleration

Compiled MEX functions can reduce integration runtime. Each integrator checks whether its required functions are available in the `elara.mex` namespace and otherwise uses the MATLAB implementation. The optional MEX functions can be compiled once with `elara.build`; solver configuration and result formats remain unchanged.

## Implementing a Custom Integrator

To add a custom integrator, derive a class from `elara.abstract.Integrator` and implement the constant `type` and the methods `simulateSystem` and `plotSolverStats`. ODE-style implementations may instead derive from `elara.abstract.IntegratorODE` to reuse the mass-matrix option and ODE statistics plot. The resulting object can then be assigned directly to `simulation.integrator`.

See also [Running Numerical Simulations](simulation.md), [Visualizing Systems and Results](visualization.md), and the simulation scripts in the `examples` folder.
