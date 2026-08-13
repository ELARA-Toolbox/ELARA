# Defining Multibody Systems

An ELARA model is an ordered array of rigid and flexible links connected in a fixed-base, open-tree topology. Branches are allowed, but closed kinematic loops are not.
Each link is attached to its parent by a one-degree-of-freedom screw joint. Alternatively, a flexible root link can represent a cantilever beam fixed directly to the base.

## Modeling Concept

ELARA uses minimal relative coordinates. The pose of every body or beam cross-section is recovered by forward kinematics along the tree and represented by an element of $\mathrm{SE}(3)$, the group of three-dimensional rigid-body poses. In compact form,

$$
g_i(q) \in \mathrm{SE}(3),
\qquad
\boldsymbol{\eta}_i = J_i(q)\dot q.
$$

Here, $g_i$ contains the absolute orientation and position of frame $i$, while the body-fixed twist $\boldsymbol{\eta}_i$ combines its angular and translational velocities expressed in that moving frame. The geometric Jacobian $J_i$ maps the generalized velocity $\dot q$ to this physical frame velocity. The generalized-coordinate vector $q = [q_1, \dots, q_n]$ comprises the screw-joint coordinates and the allowed deformations of all flexible segments. Thus, no redundant absolute poses or loop-closure constraints are introduced.

### Rigid Links

A rigid link contributes one body frame. For a screw joint with axis $X_i$ and coordinate $q_i$, its pose relative to the parent frame is

$$
g_{p(i),i}(q_i)
= g_{\mathrm{ref},i}\exp\left(\hat{X}_i q_i\right),
$$

where $g_{\mathrm{ref},i}$ is the joint transformation in the reference configuration for $q_i = 0$.
The screw axis $X_i$ defines whether the joint is a revolute, prismatic, or screw joint.

### Flexible Links

A flexible link is a spatially discretized, geometrically exact (Cosserat) beam: its cross-section frames form a serial chain, and each beam segment acts as a generalized joint. The coordinates of each segment are the allowed discrete deformations $q_i = \psi_i$.
These coordinates define the segment's overall discrete deformation $\xi_i$, from which the transformation of a discrete cross-section relative to its predecessor is computed. For a segment of length $l$ and discrete deformation $\xi_i$, the corresponding update is

$$
g_{i,i+1}=\mathrm{cay}\left(l\hat{\xi}_i\right).
$$

The hat operator denotes the matrix representation of a six-dimensional motion vector. These operations are handled internally; their practical benefit is that large three-dimensional rotations and deformations can be represented without linearizing the kinematics.

For derivations, see Chapters 3 and 4 of the dissertation and the 2024 beam-model paper listed under [Further Reading](index.md#further-reading).

## Link Array and Topology

Complete multibody systems are defined by an array of link objects.
This ordered, heterogeneous array of `elara.RigidLink` and `elara.FlexibleLink` objects defines both the bodies and their parent-child relationships.

The following rules apply during assembly:

* Exactly one link must have `parentLink = 0`.
* A parent must appear before its children in the link array.
* The topology must be acyclic; closed kinematic loops are not supported.
* One link can define a tool center point (TCP), which can be used, for example, in optimal-control applications. For simulations, this is optional.
* If the root is a flexible link, it can be defined as a cantilever beam (`isCantilever = true`), which fixes the beam's first cross-section to the base instead of adding a screw joint.

This minimal rigid-link definition illustrates the frame and joint conventions:

```matlab
link = elara.RigidLink;
link.parentLink = 0;

% Revolute joint about the joint-frame z-axis: [angular; linear]
link.jointAxis = [0; 0; 1; 0; 0; 0];
link.g_J_B = elara.SE3.matrix(eye(3), [0; 0; 0.25]);
link.g_ref = link.g_J_B;
link.jointIsActuated = true;
link.c = 0;
link.d = 1e-2;

link.m = 1;
link.J = diag([0.01, 0.01, 0.005]);
```

A complete serial-chain definition is available in `examples/example-systems/systemDef_rigid_robot.m`.

## Common Link Properties

Rigid and flexible links use the same properties for topology, screw-joint data, and the optional TCP. These common properties are inherited from `elara.abstract.Link`.

| Property | Meaning |
|---|---|
| `parentLink` | Parent link index; `0` selects the single root. |
| `jointAxis` | Six-component screw vector in the joint frame, ordered `[angular; linear]`. |
| `g_ref` | Transform from the parent link reference frame to the current body reference frame when the joint coordinate is zero. |
| `g_J_B` | Transform from the joint frame to the rigid-body frame or first beam-node frame. |
| `jointIsActuated` | Assign a scalar system input to the screw joint. It does not remove the joint when `false`. |
| `c`, `d` | Scalar screw-joint stiffness and linear dissipation. |
| `hasTCP`, `g_B_TCP` | Enable the single system TCP and place it relative to the link frame; a flexible link uses its final node. |

For a unit axis `e`, a revolute joint is specified by `[e; zeros(3,1)]`, while `[e; pitch*e]` describes a finite-pitch screw joint.

> [!NOTE]
> The current screw-exponential implementation assumes a nonzero unit angular axis; a pure prismatic axis `[zeros(3,1); e]` is currently not supported.

The common base class also stores attached-body arrays, but complete attached-inertia assembly is currently implemented for flexible links only. Additional rigid-body mass and inertia should therefore be included directly in the rigid link's `m` and `J`.

## Rigid Links

A rigid link contributes one body frame whose mass properties remain constant in that frame. `elara.RigidLink` therefore defines the mass `m` and the inertia tensor `J`. The inertia tensor is expressed about the body's center of mass and resolved in the body-fixed frame. For visualization, a bounding box is placed relative to that frame through `g_bbox`, while its positive and negative extents are stored in `bBoxSize`:

```matlab
link.g_bbox = eye(4);
link.bBoxSize = [
     0.30,  0.04,  0.04
    -0.30, -0.04, -0.04
];
```

The bounding box affects graphics only, not mass or collision calculations.

## Flexible Links

A flexible link replaces a single rigid-body frame with a chain of beam cross-sections whose relative deformations become part of the generalized coordinates. In `elara.FlexibleLink`, the beam is divided into `nSegments` equal-length segments; at least two segments are required. Its main properties are:

| Property | Meaning |
|---|---|
| `isCantilever` | Clamp the first node when this is the root link. |
| `L`, `nSegments` | Beam length and spatial discretization. |
| `Ba`, `Bc` | Orthonormal, complementary bases for allowed and constrained deformation modes. |
| `xiRef` | Reference deformation gradient, size `6`-by-`nSegments`. |
| `beamParameters` | Material, cross-section, inertia, stiffness, and damping data. |
| `tendonActuation` | Optional tendon paths and termination locations. |

The six deformation components are ordered as two bending components, torsion, two shear components, and extension. The deformation of segment $i$ is decomposed as

$$
\boldsymbol{\xi}_i
= B_a\boldsymbol{\psi}_i + B_c B_c^{\mathsf T}\boldsymbol{\xi}_{\mathrm{ref},i},
\qquad
\boldsymbol{q}_{\mathrm{ref},i}=B_a^{\mathsf T}\boldsymbol{\xi}_{\mathrm{ref},i}.
$$

The columns of $B_a$ select the deformation modes that remain free generalized coordinates, collected in $\boldsymbol{\psi}_i$. The complementary matrix $B_c$ selects modes that remain fixed at their reference values. For example, an inextensible Kirchhoff beam allows bending and torsion but constrains shear and extension. Both bases must have orthonormal columns, be mutually orthogonal, and together span all six deformation directions. Current validation uses exact matrix comparisons, so exact selection matrices such as the examples below are recommended instead of numerically approximated bases.

Common choices are:

| Beam model | Allowed basis `Ba` | Constrained basis `Bc` |
|---|---|---|
| Simo-Reissner, all six modes | `eye(6)` | `zeros(6,0)` |
| Inextensible Kirchhoff, bending and torsion | `[eye(3); zeros(3)]` | `[zeros(3); eye(3)]` |
| Inextensible Euler-Bernoulli, bending only | `[eye(2); zeros(4,2)]` | `[zeros(2,4); eye(4)]` |

A straight inextensible Kirchhoff beam can be defined as follows:

```matlab
systemsFolder = fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems");
addpath(systemsFolder)

beam = elara.FlexibleLink;
beam.parentLink = 0;
beam.isCantilever = true;
beam.jointIsActuated = false;
beam.nSegments = 5;
beam.L = 0.7;

beam.Ba = [eye(3); zeros(3)];
beam.Bc = [zeros(3); eye(3)];
beam.xiRef = repmat([0; 0; 0; 0; 0; 1], ...
    1, beam.nSegments);
beam.beamParameters = beamParams_spring_steel_round( ...
    "radius", 2e-3);
```

The assembled stress-free coordinate vector is available as `system.qRef`. The convenience methods `setJointAngles` and `setLinkDeformations` each return a full coordinate vector with all other entries set to zero. When combining joint angles with nonzero reference deformations, initialize from `qRef` or explicitly combine the nonoverlapping entries.

> [!NOTE]
> **Reference-configuration note:** `simulation.visualizeSystemRefConf` displays $q=0$. If a precurved beam has nonzero allowed components in `xiRef`, display its stress-free shape by calling `simulation.visualizeSystemConfig(simulation.system.qRef)`.

### Beam Material and Cross-Section

Once the allowed beam deformations have been selected, the physical response is determined by the material and cross-section data. `elara.BeamParameters.computeParameters` builds the generalized stiffness and inertia matrices from the following quantities:

* the elastic constants `E` and `nu`,
* the density `rho`,
* the area `A`, and
* the second moments of area `I_x` and `I_y`, and the polar second moment `J_P`.

The dimensions `height` and `width` are set separately for visualization. The vector `d` specifies Kelvin-Voigt damping coefficients for the six deformation modes; the resulting dissipative stress is proportional to the strain rate.

The geometry and material parameters can be defined using a struct and the `computeParameters` method:

```matlab
radius = 2e-3;
p = elara.BeamParameters;
p.height = 2*radius;
p.width = 2*radius;

base.E = 185e9;
base.nu = 0.3;
base.rho = 7900;
base.A = pi*radius^2;
base.I_x = pi*radius^4/4;
base.I_y = base.I_x;
base.J_P = pi*radius^4/2;

p = p.computeParameters(base);
p.d = 1e-3*ones(6,1);
```

The supplied `beamParams_ASA_round.m` and `beamParams_spring_steel_round.m` functions are useful templates.

### Attached Masses

For a flexible link, attached bodies are specified at the `nSegments + 1` beam nodes. A payload can, for example, be added at the tip as follows:

```matlab
payloadMass = 0.1;
payloadInertia = 1e-5*eye(3);  % expressed about the beam-tip node

beam.g_a = repmat(eye(4), 1, 1, beam.nSegments + 1);
beam.m_a = zeros(1, beam.nSegments + 1);
beam.M_a = zeros(6, 6, beam.nSegments + 1);

beam.m_a(end) = payloadMass;
beam.M_a(:,:,end) = blkdiag(payloadInertia, payloadMass*eye(3));
```

`M_a(:,:,i)` must be the generalized inertia expressed about beam node `i`; include the required rotation, parallel-axis, and coupling terms for an offset body. `g_a(:,:,i)` records the center-of-mass transform used for gravity but does not transform `M_a` during assembly.

### Tendon Actuation

The toolbox supports tendon actuation for flexible links, enabling models of tendon-actuated soft robots and continuum manipulators.
Any number of tendons with arbitrary paths along the backbone can be specified.

A tendon path is a function of arc length `s` that returns the tendon position relative to the beam backbone. The model stores one path and one termination length for each tendon. The required derivatives may be provided directly or generated with the symbolic helper:

```matlab
r = 0.02;
beam.tendonActuation.x_td_funs = {
    @(s) [ r; 0; 0]
    @(s) [-r; 0; 0]
};
beam.tendonActuation.LTermination = [beam.L; beam.L];
beam.tendonActuation = ...
    beam.tendonActuation.getSymbolicPathDerivatives;
```

Each tendon-actuated flexible link uses one input per tendon. A specified termination length is snapped to the nearest beam node, so the beam discretization should contain a node sufficiently close to each intended termination point. Straight and helical full-length paths are illustrated in `systemDef_continuum_manipulator.m`. The continuum-robot reference listed under [Further Reading](index.md#further-reading) provides additional modeling context.

## Assembling a System

Once the link array is complete, it is converted into the compact system representation used by the numerical algorithms. Construction performs four main tasks:

* validating the link definitions,
* building the link and frame graphs,
* assigning generalized coordinates and inputs, and
* assembling the inertial, stiffness, damping, actuation, and TCP data.

Numeric and symbolic systems are then constructed with:

```matlab
systemNum = elara.SystemNum(links);
systemSym = elara.SystemSym(links);
```

`elara.Simulation(links)` constructs `SystemNum` automatically. `elara.ocp.Problem(links)` constructs both representations and uses CasADi when building symbolic optimization functions.

The most useful assembled properties fall into three groups:

* system dimensions: `nLinks`, `nJoints`, `nFrames`, `nDoF`, and `nInputs`;
* topology mapping: `linkFrameIndices`; and
* reference and constitutive data: `qRef`, `cSys`, and `dSys`.

The main high-level operations are:

```matlab
g = systemNum.computeFwdKin(q);
J = systemNum.computeGeomJacobian(q);
M = systemNum.computeMassMatrix(q);
B = systemNum.computeInputMatrix(q);

theta = systemNum.getJointAngles(q);
xi = systemNum.getLinkDeformations(q, iLink);
```

Many of these methods have variants with the suffix `Fast`.
These variants accept precomputed intermediate kinematics—for example, the relative joint transformations returned by `g_rel = systemNum.computeJointTransformations(q)`—and are intended for performance-sensitive internal or advanced use.
They can avoid repeated computation of relative joint transformations in complex calculations, such as the equations of motion, where several kinematic quantities depend on the same intermediate variables.


`SystemNum` and `SystemSym` share the core system interface but use different internal representations. In particular, the numeric and symbolic systems stored by an `elara.ocp.Problem` are independent copies: modifying the original links or either assembled system after construction does not update the others. Reconstruct the problem after changing model parameters.

The assembled link and frame topology can be inspected with `elara.plot.systemGraphs(systemNum)`.

## Related Examples

The example-system definitions illustrate the most common link combinations:

* `systemDef_rigid_robot.m` — three rigid screw-joint links and a TCP.
* `systemDef_cantilever_system.m` — a cantilever beam followed by two rigid links.
* `systemDef_rigid_flexible_robot.m` — mixed rigid-flexible serial robot.
* `systemDef_continuum_manipulator.m` — attached payload and tendon actuation.

These functions are designed to be copied and adapted when a new model is started.

See also [Running Numerical Simulations](simulation.md), [Solving Optimal-Control Problems](optimal_control.md), and [Visualizing Systems and Results](visualization.md).
