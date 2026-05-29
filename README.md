# MATLAB Toolbox ELARA

ELARA - short for "Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control" - 
is a MATLAB Toolbox for the simulation and optimal control of rigid-flexible mechanical systems using structure-preserving Lie-group methods. 


## Toolbox Overview

Type of multibody systems,
examples: Rigid manipulators, robots with elastic links,
continuum manipulators

Joint types: lower-pair/screw joints: revolute, prismatic, screw

Link types: Rigid, flexible (modeled as geometrically exact beams)

Actuation: Actuated joints, tendon actuation



## Getting Started

Installation

CasADi for optimal control

Code generation (optional, but recommended):
Run `build.m`

## Included Example Scripts

### Multibody simulation

### Optimal Control


## System Definition

Joint definition,
joint actuation

### MBLinkDefinition


#### Rigid Links

Transformations, inertia properties etc.

#### Flexible Links

Included deformation modes, nr. of segments

Transformations

Actuation

### MBSystem Class

with Num and Sym variants

Symbolic variant provides a fully CasADi compatible definition of all functions to be used with optimal control, parameter identification, and others


## Simulation / Time Integration

### MBSimulation Class

### MBSimIntegrator Class

##### Using MATLAB ODE Methods

##### Using the LGVI

Tuning Options: Step Size $h$, other parameters...

## Optimal Control

via CasADi

### OCPDefinition Class
for optimal control


### OCPDiscretization Class

## References and Further links

Continuum manipulator: HPK26 paper

Beam model: HK24 Paper

See also

Müller, 2018 papers
Murray 1994 book



## Limitations and Possible Future Extensions

 * Add recursive algorithms for continuous and discrete dynamics (forward and inverse)
 * Add contact forces, e.g., with environment or grippers
 * Add constraints for systems with closed kinematic loops
 * Add support for floating-base systems
 * Add URDF interface (define systems via URDF files)