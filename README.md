<div align="center">
  <img src="doc/.assets/banner.png" alt="Elara: Efficient Lie-group Algorithms for Flexible Robot Analysis and Control" style="width:100%;height:auto;max-width:100%;"/>
</div>

[![View ELARA - Flexible Robot Simulation and Control Toolbox on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/183990-elara-flexible-robot-simulation-and-control-toolbox)
[![MATLAB](https://img.shields.io/badge/language-MATLAB-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![MATLAB Version](https://img.shields.io/badge/MATLAB-R2025b%2B-blue)
![GitHub Repo stars](https://img.shields.io/github/stars/ELARA-Toolbox/ELARA?style=social)

# ELARA Simulation and Optimal Control Toolbox

ELARA, short for **Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control**, is a MATLAB toolbox for the simulation and optimal control of robotic multibody systems consisting of rigid and highly flexible links.
It combines Lie-group kinematics on SE(3), rigid multibody dynamics, geometrically exact beam models, structure-preserving variational integration, and CasADi-based direct optimal control.

## Features

- Supports rigid-flexible multibody systems with open-tree kinematics consisting of rigid links, flexible beam links, and one-DoF revolute or finite-pitch screw joints
- Actuation via actuated screw joints or tendon actuation for continuum manipulators
- Analytic kinematics and differential kinematics on SE(3), including forward kinematics, geometric Jacobians, and derivatives of Jacobians
- Time integration with highly efficient Lie-group variational integrator or any standard ODE solver via general interface (including all MATLAB ODE solvers)
- CasADi-backed symbolic implementation for direct optimal control
- Direct optimal control with various time discretizations, including variational DMOC, Runge-Kutta (2nd and 4th order), and implicit-midpoint discretizations
- Extensive plotting, visualization, and animation functionality
- Optional MEX implementations of performance-critical simulation functions through MATLAB Coder

## Requirements

- MATLAB `R2025b` or later
- Signal Processing Toolbox: required by the public SO(3) and SE(3) exponential-map functions used in the core simulation workflow

Optional dependencies:

- CasADi for MATLAB: required for optimal control or other optimization-related tasks (tested with `3.7.2`)
- MATLAB Coder and a supported C++ compiler for optional MEX generation
- Symbolic Math Toolbox for tendon-path derivatives
- Robotics System Toolbox, Optimization Toolbox, and Mapping Toolbox for selected optimal-control helpers; see the [documentation](elara-toolbox/doc/index.md#requirements-and-setup) for feature-level details

### Toolbox installation

There are several convenient ways to install ELARA:

- MATLAB Add-On Explorer (recommended): ELARA can be found by searching for its name and selecting **Install**. The same installation is available through [MATLAB File Exchange](https://www.mathworks.com/matlabcentral/fileexchange/183990-elara-flexible-robot-simulation-and-control-toolbox).
- Packaged toolbox file: download the toolbox file `elara-toolbox.mltbx` from the release page, then double-click on it or run `matlab.addons.install("elara-toolbox.mltbx")`.
- Local repository clone: the folder `elara-toolbox`, including its subfolders, can be added to the MATLAB path.

### Optional components
- For optimal control, install CasADi from <https://web.casadi.org/> and add it to the MATLAB path
- For code generation (to generate MEX files), install a compatible C++ compiler and setup MATLAB Coder using `mex -setup C++`

### Verify installation and compile MEX functions
1. In the MATLAB console, run `elara.setup` to validate the installation of the toolbox and the optional components.
2. With MATLAB Coder and a compatible C++ compiler available, `elara.build` compiles the performance-critical functions into the `elara.mex` namespace.
3. Run `elara.setup` again to verify that the MEX functions are available. The toolbox will then automatically use functions such as `elara.mex.integrateVIBroyden_mex` instead of their MATLAB implementations.

## Getting Started
A [live quick-start guide](elara-toolbox/doc/GettingStarted.mlx) explains the installation steps and core functionality.

The toolbox includes several examples for the simulation and optimal control of mechanical systems available in the `examples` folder.

#### Simulation Examples
The simulation examples demonstrate the standard workflow to simulate the dynamical behavior of mechanical systems:

1. Define the system's individual links using an array of `elara.RigidLink` and `elara.FlexibleLink` objects.
2. Define a `elara.Simulation` object that handles the complete simulation workflow.
The simulation object contains an `elara.SystemNum` object that is assembled from the links and stores the complete multibody system.
3. Set the simulation parameters in the `parameters` field of the simulation object for parameters such as initial conditions, inputs, gravity, and final time.
4. Select an integrator such as `elara.integration.VIBroyden` or `elara.integration.ODEDirect`.
5. Run `simulateSystem`, then use plotting or animation helpers for post-processing.

#### Optimal Control Examples

The optimal-control examples demonstrate:

1. Creating an `elara.ocp.Problem` object, which holds the system definition and all parameters of the optimal control problem.
2. Using the symbolic system object `elara.SystemSym` within CasADi.
3. Selecting an OCP discretization such as `elara.ocp.DiscretizationVI` or `elara.ocp.DiscretizationRK`.
4. Initializing the NLP solver with the `initSolver` method.
5. Solving with the `solve` method and post-processing the resulting trajectory.


## Documentation

Details on the toolbox can be found in its documentation:

* [Getting Started](elara-toolbox/doc/index.md)
* [Defining Multibody Systems](elara-toolbox/doc/system_definition.md)
* [Running Numerical Simulations](elara-toolbox/doc/simulation.md)
   * [Choosing an Integration Method](elara-toolbox/doc/integration.md)
* [Solving Optimal-Control Problems](elara-toolbox/doc/optimal_control.md)
   * [Choosing an OCP Discretization](elara-toolbox/doc/ocp_discretization.md)
* [Visualizing Systems and Results](elara-toolbox/doc/visualization.md)

The documentation is also available in the MATLAB documentation viewer.
To access it, open the documentation (e.g., by typing `doc`) and click `Supplemental Software`.


## Citation

Research making use of the ELARA toolbox may cite the following papers:

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

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

Third-party components in `elara-toolbox/third-party/` are redistributed under their respective individual licenses.
In particular, this concerns the following packages:

 - `vert2lcon.m` from the [N-dimensional Convex Polyhedra](https://www.mathworks.com/matlabcentral/fileexchange/30892-analyze-n-dimensional-convex-polyhedra) package by Matt Jacobson. Available on MATLAB File Exchange. Included version: 1.9.0.2.

## Authors

- Maximilian Herrmann (TUM, Chair of Automatic Control)
- Leander Pfeiffer (TUM, Chair of Automatic Control)

The core toolbox was developed by Maximilian Herrmann. Leander Pfeiffer provided helpful input and contributions in the final development stages.
Moreover, Philipp Tarbiat, Tobias Farger, and Akash Cheriath contributed helpful input and code snippets during their student projects at the chair.
These contributions are marked in the comments.

## Developer documentation

For release, the toolbox is packaged with the script `tools/packageToolbox.m`, which creates the corresponding `mltbx` file.
The version number in the script must be incremented manually.

## Contact & Support

For questions, issues, or feature requests:

- Open an issue on GitHub
- Contact: [maximilian.herrmann@tum.de, leander.pfeiffer@tum.de]
