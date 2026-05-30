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

- CasADi for MATLAB: required for optimal control or other optimization-related tasks (tested with `3.7.2`)
- MATLAB Coder and C++ compiler for optional MEX generation

### Toolbox installation
- Recommended: Installation via the packaged toolbox file 
Double-click on `elara-toolbox.mltbx` or run `matlab.addons.install("elara-toolbox.mltbx")`

- Alternatively, clone the repository locally and add the folder `elara-toolbox` (including subfolders) to your MATLAB path

### Optional components
- For optimal control, install CasADi from <https://web.casadi.org/> and add it to the MATLAB path
- For code generation (to generate MEX files), install a compatible C++ compiler and setup MATLAB Coder using `mex -setup C++`

### Verify installation and compile MEX functions
1. In the MATLAB console, run `elara.setup` to validate the installation of the toolbox and the optional components. The command displays, whether all components are installed correctly.
2. If you have MATLAB Coder and a compatible C++ compiler installed, run `elara.build`, which will compile all required MEX files. The compiled files are stored in the `/build` directory.
3. You can again run `elara.setup` to verify that the MEX files are available on the path. The toolbox will now automatically use the compiled functions instead of the slower MATLAB functions.

## Quick Start
A quick start guide is available under `doc/QuickStart.mlx`, explaining the installation steps and core functionality.

### Examples
The toolbox includes several examples for the simulation and optimal control of mechanical systems available in the `examples` folder.
The simulation examples demonstrate the standard workflow to simulate the dynamical behavior of mechanical systems:

1. Define the system's links using `MBLinkDefinition` objects.
2. Create an `MBSimulation` object that stores the system definition and all simulation parameters.
3. Set `MBSim.simPars` for initial conditions, inputs, gravity, and final time.
4. Select a solver such as `MBSimIntegratorVarIntBroyden` or `MBSimIntegratorODEDirect`.
5. Run `simulateSystem`, then use plotting or animation helpers for post-processing.

The optimal-control examples demonstrate:

1. Creating an `OCPDefinition`.
2. Using `MBSystemSym(links)` for CasADi-compatible dynamics.
3. Selecting an OCP discretization such as `OCPIntegratorVI` or `OCPIntegratorRK`.
4. Initializing the NLP solver with `initSolver`.
5. Solving with `solve` and post-processing the resulting trajectory.

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




## Citation

If you use the Elara toolbox in your research, please cite the following papers:

**BibTeX:**

```bibtex
@article{HK24,
  title = {Relative-Kinematic Formulation of Geometrically Exact Beam Dynamics Based on {{Lie}} Group Variational Integrators},
  author = {Herrmann, Maximilian and Kotyczka, Paul},
  year = 2024,
  month = dec,
  journal = {Computer Methods in Applied Mechanics and Engineering},
  volume = {432},
  pages = {117367},
  issn = {00457825},
  doi = {10.1016/j.cma.2024.117367},
}
@inproceedings{HPK26,
  title = {Discrete {{Geometric Modeling}} and {{Extended State Estimation}} of {{Continuum Robots}}},
  booktitle = {IFAC World Congress},
  author = {Herrmann, Maximilian and Pfeiffer, Leander and Kotyczka, Paul},
  year = 2026,
  address = {Busan},
  }
```

**APA:**

> Herrmann, M., & Kotyczka, P. (2024). Relative-kinematic formulation of geometrically exact beam dynamics based on Lie group variational integrators. Computer Methods in Applied Mechanics and Engineering, 432, 117367. https://doi.org/10.1016/j.cma.2024.117367

> Herrmann, M., Pfeiffer, L., & Kotyczka, P. (2026). Discrete Geometric Modeling and Extended State Estimation of Continuum Robots. IFAC World Congress.

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## Authors

- Maximilian Herrmann (TUM, Chair of Automatic Control)
- Leander Pfeiffer (TUM, Chair of Automatic Control)

The core toolbox was developed by Maximilian Herrmann. Leander Pfeiffer provided helpful input and contributions in the final development stages.
Moreover, Philipp Tarbiat, Tobias Farger, and Akash Cheriath contributed helpful input and code snippets during their student projects at the chair.
These contributions are marked in the comments.

## Contact & Support

For questions, issues, or feature requests:

- Open an issue on GitHub
- Contact: [maximilian.herrmann@tum.de, leander.pfeiffer@tum.de]
