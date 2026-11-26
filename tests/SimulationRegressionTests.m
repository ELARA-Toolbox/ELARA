function tests = SimulationRegressionTests
% Release regressions for beam assembly, integration status, and visualization.
% makeBeam supplies a straight flexible beam with analytically known mass and
% inertia. makeSimulation supplies an actuated, undamped two-link pendulum;
% most VI tests deliberately restrict its solver to provoke nonconvergence.
% Interpolation and display tests instead use prescribed joint trajectories.
% Status meanings: 0 = failed/incomplete, 1 = target tolerance met,
% 2 = VI completed using the relaxed toleranceLimit.
% Numerical calls use MEX when available and otherwise the MATLAB functions.
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Make the supplied pendulum model factory available to all tests below.
addpath(fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems"));
end

%% Distributed beam mass, spatial inertia, and payload assembly

function testJointedBeamMassAndInertia(testCase)
% A beam with a revolute base joint retains both physical endpoint nodes.
% For coarse and finer meshes, summed nodal mass/inertia must equal the
% per-length values times L; the tip must receive only a half-segment share.
for nSegments = [2, 5]
    beam = makeBeam(nSegments, false, 0);
    system = elara.SystemNum(beam);
    verifyEqual(testCase, sum(system.frames.m), ...
        beam.L * beam.beamParameters.m, 'AbsTol', 1e-12);
    verifyEqual(testCase, sum(system.frames.MGen, 3), ...
        beam.L * beam.beamParameters.Mgen, 'AbsTol', 1e-12);
    verifyEqual(testCase, system.frames.MGen(:,:,end), ...
        beam.L/(2*nSegments) * beam.beamParameters.Mgen, 'AbsTol', 1e-12);
end
end

function testCantileverOmitsOnlyTheFixedBaseNode(testCase)
% The same beam is now clamped. Its fixed base has no moving frame, so only
% that node's half-segment contribution is excluded from the assembled data.
% Check both mass and spatial inertia for two different mesh sizes.
for nSegments = [2, 5]
    beam = makeBeam(nSegments, true, 0);
    system = elara.SystemNum(beam);
    movingLength = beam.L - beam.L/(2*nSegments);
    verifyEqual(testCase, sum(system.frames.m), ...
        movingLength * beam.beamParameters.m, 'AbsTol', 1e-12);
    verifyEqual(testCase, sum(system.frames.MGen, 3), ...
        movingLength * beam.beamParameters.Mgen, 'AbsTol', 1e-12);
end
end

function testPayloadIsAddedOnce(testCase)
% Add a 0.3 kg tip payload to a five-segment beam, testing both base types.
% Distributed inertia follows the nodal weights, but the payload's complete
% mass and inertia must be added once, without the tip's half-weight factor.
payloadMass = 0.3;
for isCantilever = [false, true]
    beam = makeBeam(5, isCantilever, payloadMass);
    system = elara.SystemNum(beam);
    movingLength = beam.L - isCantilever*beam.L/(2*beam.nSegments);
    verifyEqual(testCase, sum(system.frames.m), ...
        movingLength*beam.beamParameters.m + payloadMass, 'AbsTol', 1e-12);
    verifyEqual(testCase, sum(system.frames.MGen, 3), ...
        movingLength*beam.beamParameters.Mgen + beam.M_a(:,:,end), ...
        'AbsTol', 1e-12);
end
end

function testBeamAttachedToARigidLink(testCase)
% Attach the jointed beam to a one-link rigid pendulum instead of ground.
% Sum only the beam's frame block to check that its distributed mass remains
% correct when the beam is assembled as a child of another moving link.
root = systemDef_planarNLinkPendulum("nLinks", 1);
beam = makeBeam(5, false, 0);
beam.parentLink = 1;
system = elara.SystemNum([root; beam]);
frames = system.linkFrameIndices(1,2):system.linkFrameIndices(2,2);
verifyEqual(testCase, sum(system.frames.m(frames)), ...
    beam.L*beam.beamParameters.m, 'AbsTol', 1e-12);
end

%% Variational integration: failure reporting and accepted trajectory prefixes

function testInitialStepFailureReturnsStatus(testCase)
% Gravity acts on the displaced two-link pendulum. One Broyden iteration
% cannot meet the strict fixture tolerances, so only the initial pose remains.
% The shared assertion helper checks the warning, status 0, and retained data.
simulation = makeSimulation;
verifyIntegrationFailure(testCase, simulation, 1);
end

function testLaterStepFailureReturnsStatus(testCase)
% Start the two-link pendulum at rest with gravity off. Input is zero at t=0
% and nonzero at t=h, so the first interval stays at equilibrium and the
% second solve fails. The accepted prefix must end at h, before that interval.
simulation = makeSimulation;
simulation.parameters.g = 0;
simulation.parameters.q0 = zeros(2,1);
simulation.parameters.uSampleTimes = [0; simulation.integrator.h; ...
    simulation.parameters.tEnd];
simulation.parameters.uSampleValues = [zeros(2,1), [0.3; 0.5], [0.3; 0.5]];
verifyIntegrationFailure(testCase, simulation, 2);
end

function testFinalStepFailureReturnsStatus(testCase)
% Keep the gravity-free pendulum unforced through t=2h, then ramp the torques
% to nonzero values at t=3h. Only the fourth and final interval should fail.
% Its attempted endpoint must not make this look like a completed simulation.
simulation = makeSimulation;
simulation.parameters.g = 0;
simulation.parameters.q0 = zeros(2,1);
h = simulation.integrator.h;
simulation.parameters.uSampleTimes = [0; 2*h; 3*h; simulation.parameters.tEnd];
simulation.parameters.uSampleValues = [zeros(2,2), [0.3; 0.5], [0.3; 0.5]];
verifyIntegrationFailure(testCase, simulation, 4);
end

function testNonfiniteResidualReturnsStatus(testCase)
% Inject NaN motor torques into the two-link model to produce a nonfinite
% DEL residual. Numerical breakdown must return status 0 and a warning,
% retaining the finite initial pose instead of throwing a solver error.
simulation = makeSimulation;
simulation.parameters.uConst = nan(2,1);
verifyIntegrationFailure(testCase, simulation, 1);
end

function testRawInitialFailureReturnsWithoutWarning(testCase)
% Bypass VIBroyden's post-processing and call the numerical kernel directly.
% The default pendulum fails its initial solve, but the kernel must return
% its initial pose and failed-attempt diagnostics without issuing a warning.
simulation = makeSimulation;
[previousMessage, previousId] = lastwarn;
warningCleanup = onCleanup(@() lastwarn(previousMessage, previousId));
lastwarn('');
results = integrateRaw(simulation);
[~, warningId] = lastwarn;
verifyEmpty(testCase, warningId);
verifyEqual(testCase, results.tout, 0);
verifyEqual(testCase, results.q, simulation.parameters.q0);
verifyEqual(testCase, results.g, ...
    simulation.system.computeFwdKin(simulation.parameters.q0));
verifyEqual(testCase, results.solverExitFlag, 1);
verifyGreaterThan(testCase, results.solverResidual, ...
    simulation.integrator.toleranceLimit);
verifyTrue(testCase, all(isnan(results.q_dot), 'all'));
end

function testRawLaterFailureKeepsTheAcceptedPrefix(testCase)
% As in the later-failure test, the first interval is stationary and accepted;
% torque at t=h makes the second solve fail. Inspect the raw kernel output:
% poses stop at h, its final node retains the failed attempt's flag, and
% interval velocities there are NaN because no accepted next pose exists.
simulation = makeSimulation;
simulation.parameters.g = 0;
simulation.parameters.q0 = zeros(2,1);
simulation.parameters.uSampleTimes = [0; simulation.integrator.h; ...
    simulation.parameters.tEnd];
simulation.parameters.uSampleValues = [zeros(2,1), [0.3; 0.5], [0.3; 0.5]];
results = integrateRaw(simulation);
verifyEqual(testCase, results.tout, [0; simulation.integrator.h]);
verifyEqual(testCase, results.q, zeros(2,2));
verifyEqual(testCase, results.solverExitFlag, [0, 1]);
verifyTrue(testCase, all(isnan(results.q_dot(:,end))));
verifyTrue(testCase, all(isnan(results.eta(:,:,end)), 'all'));
end

function testFailureStatusIsAvailableWithWarningsDisabled(testCase)
% Disable the failure warning for the default failing pendulum and request
% accurate timing. Status 0 must still be available to programmatic callers.
% This also exercises the path that skips benchmarking a failed simulation.
simulation = makeSimulation;
simulation.integrator.accurateTiming = true;
state = warning('query', 'elara:IntegrationFailed');
warningCleanup = onCleanup(@() warning(state));
warning('off', 'elara:IntegrationFailed');
simulation = simulation.simulateSystem;
verifyEqual(testCase, simulation.results.simulationStatus, 0);
verifyGreaterThanOrEqual(testCase, simulation.results.computationTime, 0);
end

function testRelaxedToleranceRemainsAcceptable(testCase)
% Simulate one interval with the strict target unchanged but a generous hard
% limit. The pendulum's residual misses the target yet meets toleranceLimit:
% keep the computed coordinates and report overall status 2 despite flag 1.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.integrator.toleranceLimit = 1;
simulation = simulation.simulateSystem;
verifyEqual(testCase, simulation.results.tout(end), ...
    simulation.parameters.tEnd, 'AbsTol', 1e-12);
verifyEqual(testCase, simulation.results.solverExitFlag(1), 1);
verifyGreaterThan(testCase, simulation.results.solverResidual(1), ...
    simulation.integrator.tolerance);
verifyLessThanOrEqual(testCase, simulation.results.solverResidual(1), ...
    simulation.integrator.toleranceLimit);
verifyTrue(testCase, all(isfinite(simulation.results.q), 'all'));
verifyTrue(testCase, all(isnan(simulation.results.q_dot(:,end))));
verifyTrue(testCase, all(isnan(simulation.results.eta(:,:,end)), 'all'));
verifyEqual(testCase, simulation.results.simulationStatus, 2);
end

function testSuccessfulIntegrationReachesTheHorizon(testCase)
% Give the displaced pendulum enough iterations to meet realistic tolerances.
% Threshold zero exercises Jacobian refresh after each step with iterations.
% All four intervals must complete with status 1 and converged step flags;
% exclude the final node's NaN padding when checking those flags.
simulation = makeSimulation;
simulation.integrator.maxIterations = 100;
simulation.integrator.JacobianIterationThreshold = 0;
simulation.integrator.tolerance = 1e-10;
simulation.integrator.toleranceLimit = 1e-8;
simulation = simulation.simulateSystem;
verifyEqual(testCase, simulation.results.tout(end), ...
    simulation.parameters.tEnd, 'AbsTol', 1e-12);
verifyTrue(testCase, all(isfinite(simulation.results.q), 'all'));
verifyTrue(testCase, all(simulation.results.solverExitFlag(1:end-1) == 0));
verifyEqual(testCase, simulation.results.simulationStatus, 1);
end

function testZeroStepGridIsRejectedBeforeAllocation(testCase)
% tEnd=h/4 rounds to zero integration intervals although both inputs are
% positive. This invalid grid must be rejected before indexing preallocated
% output arrays; the former compiled path could otherwise corrupt memory.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h/4;
verifyError(testCase, @() simulation.simulateSystem, 'elara:InvalidTimeGrid');
end

function testToleranceBoundaries(testCase)
% Measure the pendulum's residual after one iteration, then build threshold
% cases from that measured value. This tests exact equality without relying
% on a hard-coded residual that could differ between MATLAB and MEX.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.integrator.tolerance = 1e-20;
simulation.integrator.toleranceLimit = 1;
simulation = simulation.simulateSystem;
residual = simulation.results.solverResidual(1);
verifyGreaterThan(testCase, residual, 0);

% Columns are [target tolerance, hard limit, expected overall status].
% Rows test equality at the target (1), equality at the relaxed limit (2),
% and a hard limit just below the residual (0), using eps for that last gap.
cases = [residual, residual, 1; residual/2, residual, 2; ...
    residual/2, residual-eps(residual), 0];
for k = 1:size(cases,1)
    candidate = simulation;
    candidate.integrator.tolerance = cases(k,1);
    candidate.integrator.toleranceLimit = cases(k,2);
    candidate = simulateIgnoringFailureWarning(candidate);
    verifyEqual(testCase, candidate.results.solverResidual(1), residual);
    verifyEqual(testCase, candidate.results.simulationStatus, cases(k,3));
end
end

function testZeroResidualSatisfiesToleranceDespiteSolverFlag(testCase)
% At rest with gravity off, the pendulum already has zero DEL residual.
% With zero iterations allowed, the forced initial solve retains local flag 1.
% Overall success must nevertheless be determined from its satisfied residual.
simulation = makeSimulation;
simulation.parameters.g = 0;
simulation.parameters.q0 = zeros(2,1);
simulation.integrator.maxIterations = 0;
simulation = simulation.simulateSystem;
verifyEqual(testCase, simulation.results.solverExitFlag(1), 1);
verifyEqual(testCase, simulation.results.solverResidual(1), 0);
verifyEqual(testCase, simulation.results.simulationStatus, 1);
end

function testHardLimitFailureWithFullTrajectoryAndSolverFlagZero(testCase)
% Deliberately set the target above the hard limit for one pendulum interval.
% The kernel can report local convergence and return the full sample count
% while exceeding the hard limit. Neither observation may imply status 1.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.integrator.tolerance = 1;
simulation = simulateIgnoringFailureWarning(simulation);
verifyEqual(testCase, numel(simulation.results.tout), 2);
verifyEqual(testCase, simulation.results.solverExitFlag(1), 0);
verifyGreaterThan(testCase, simulation.results.solverResidual(1), ...
    simulation.integrator.toleranceLimit);
verifyEqual(testCase, simulation.results.simulationStatus, 0);
end

function testInconsistentTolerancesDoNotReportSuccess(testCase)
% Both thresholds are loose enough for the one-step pendulum residual, but
% tolerance > toleranceLimit is an invalid pair. This isolates the ordering
% check from ordinary residual-based nonconvergence and must give status 0.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.integrator.tolerance = 1;
simulation.integrator.toleranceLimit = 0.1;
simulation = simulateIgnoringFailureWarning(simulation);
verifyEqual(testCase, simulation.results.simulationStatus, 0);
end

%% Setting validation without assembling or simulating a physical system

function testPositiveSettingsRejectInvalidValues(testCase)
% Check assignment directly on both the public integrator and kernel config,
% so neither entry point can bypass validation of h or the two tolerances.
% Positive infinity passes positivity but must still fail the finite check.
objects = {elara.integration.VIBroyden, elara.internal.integration.VIBroydenConfig};
for k = 1:numel(objects)
    obj = objects{k};
    for propertyName = ["h", "tolerance", "toleranceLimit"]
        for value = [-1, 0, NaN, -Inf]
            verifyError(testCase, @() assignSetting(obj, propertyName, value), ...
                'MATLAB:validators:mustBePositive');
        end
        verifyError(testCase, @() assignSetting(obj, propertyName, Inf), ...
            'MATLAB:validators:mustBeFinite');
    end
end
end

function testIterationSettingsRejectInvalidValues(testCase)
% Both iteration settings represent counts: reject fractions, negative
% values, and nonfinite values on both classes. Preserve zero for skipping
% Broyden iterations or selecting the minimum Jacobian refresh threshold.
objects = {elara.integration.VIBroyden, elara.internal.integration.VIBroydenConfig};
for k = 1:numel(objects)
    obj = objects{k};
    for propertyName = ["maxIterations", "JacobianIterationThreshold"]
        for value = [-1, NaN, -Inf]
            verifyError(testCase, @() assignSetting(obj, propertyName, value), ...
                'MATLAB:validators:mustBeNonnegative');
        end
        verifyError(testCase, @() assignSetting(obj, propertyName, Inf), ...
            'MATLAB:validators:mustBeFinite');
        verifyError(testCase, @() assignSetting(obj, propertyName, 1.5), ...
            'MATLAB:validators:mustBeInteger');
        obj.(propertyName) = 0;
        verifyEqual(testCase, obj.(propertyName), 0);
    end
end
end

function testTrapezoidalWeightValidation(testCase)
% Check the internal generalized trapezoidal weight independently of a model.
% Accept both endpoints and interior weights in [0,1], including the public
% integrator's presets 0 and 0.5. Reject just-outside and nonfinite values.
config = elara.internal.integration.VIBroydenConfig;
for value = [0, 0.25, 0.5, 1]
    config.aTrapez = value;
    verifyEqual(testCase, config.aTrapez, value);
end
for value = [-eps, 1+eps]
    verifyError(testCase, @() assignSetting(config, "aTrapez", value), ...
        'MATLAB:validators:mustBeInRange');
end
for value = [NaN, Inf, -Inf]
    verifyError(testCase, @() assignSetting(config, "aTrapez", value), ...
        'MATLAB:validators:mustBeFinite');
end
end

%% ODE interfaces: the same completion-status contract as the VI wrapper

function testObjectBasedODESuccessStatus(testCase)
% Replace the restrictive VI with object-based ode15s on the two-link model.
% Exercise both explicit derivatives and M(x)*x_dot=f(x) mass-matrix form;
% each run must reach the requested 0.2 s horizon and report status 1.
% ode15s accepts the ODE object's default "maybe" mass singularity setting;
% ode45 would warn about assuming a nonsingular matrix in this branch.
for useMassMatrix = [false, true]
    simulation = makeSimulation;
    simulation.integrator = elara.integration.ODEDirect;
    simulation.integrator.odeObject.Solver = "ode15s";
    simulation.integrator.useMassMatrixForm = useMassMatrix;
    simulation.integrator.showConsoleOutput = false;
    simulation = simulation.simulateSystem;
    verifyEqual(testCase, simulation.results.simulationStatus, 1);
    verifyEqual(testCase, simulation.results.tout(end), ...
        simulation.parameters.tEnd, 'AbsTol', 1e-12);
end
end

function testFunctionBasedODESuccessStatus(testCase)
% Repeat the two-link completion check through the classic ode45 function
% interface, covering both equation forms. Status must have the same meaning
% as for the object-based ODE interface and a successful VI run.
for useMassMatrix = [false, true]
    simulation = makeSimulation;
    simulation.integrator = elara.integration.ODEDirectFunctionBased;
    simulation.integrator.useMassMatrixForm = useMassMatrix;
    simulation.integrator.showConsoleOutput = false;
    simulation = simulation.simulateSystem;
    verifyEqual(testCase, simulation.results.simulationStatus, 1);
    verifyEqual(testCase, simulation.results.tout(end), ...
        simulation.parameters.tEnd, 'AbsTol', 1e-12);
end
end

function testEarlyODEReturnDoesNotReportSuccess(testCase)
% A terminal event stops the two-link simulation at 0.075 s, before tEnd.
% This produces an incomplete ODE trajectory deterministically, without
% provoking numerical breakdown: its overall completion status must be 0.
simulation = makeSimulation;
simulation.integrator = elara.integration.ODEDirectFunctionBased;
simulation.integrator.solverOptions = odeset('Events', @stopEarly);
simulation.integrator.showConsoleOutput = false;
simulation = simulation.simulateSystem;
verifyEqual(testCase, simulation.results.simulationStatus, 0);
verifyEqual(testCase, simulation.results.tout(end), 0.075, 'AbsTol', 1e-8);
end

function [value, isterminal, direction] = stopEarly(t, ~)
% Stop ode45 on the increasing zero crossing of t-0.075, independently of
% the pendulum state, so the early-return test has a known stopping time.
value = t - 0.075;
isterminal = 1;
direction = 1;
end

%% Prescribed trajectories: interpolation and single-coordinate display

function testInterpolationOverNonuniformTimes(testCase)
% Use one- and two-link pendulums for geometry, prescribing q instead of
% integrating it. Compare against hand-computed linear interpolation on a
% nonuniform time grid, then verify poses by direct forward kinematics.
% The one-link case checks interpolation with only one generalized coordinate.
for nLinks = [1, 2]
    system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", nLinks));
    results = elara.SimulationResults;
    results.tout = [0; 0.2; 0.5];
    q = [0.1, 0.5, -0.1; -0.2, 0.2, 0.6];
    results.q = q(1:nLinks,:);
    expected = [0.1, 0.3, 0.5, 0.2, -0.1; -0.2, 0, 0.2, 0.4, 0.6];
    times = [0; 0.1; 0.2; 0.35; 0.5];
    [g, qQuery] = elara.internal.simulation.interpolateResults( ...
        system, results, times);
    verifySize(testCase, qQuery, [nLinks, numel(times)]);
    verifyEqual(testCase, qQuery, expected(1:nLinks,:), 'AbsTol', 1e-12);
    for k = 1:numel(times)
        verifyEqual(testCase, g(:,:,:,k), ...
            system.computeFwdKin(expected(1:nLinks,k)), 'AbsTol', 1e-12);
    end
end
end

function testSingleCoordinateSnapshotsAndAnimation(testCase)
% Give a one-link pendulum a prescribed constant-speed joint trajectory.
% Exercise snapshots and animation through their public helpers, which use
% interpolation internally. Hidden figures avoid interactive windows; this
% checks that rendering completes and creates figures, not their appearance.
previousVisibility = get(groot, "defaultFigureVisible");
visibilityCleanup = onCleanup( ...
    @() set(groot, "defaultFigureVisible", previousVisibility));
set(groot, "defaultFigureVisible", "off");

simulation = elara.Simulation( ...
    systemDef_planarNLinkPendulum("nLinks", 1), "displayInfo", false);
times = [0; 0.05; 0.1];
simulation.results = elara.SimulationResults.fromStateTrajectory( ...
    simulation.system, times, [0, 0.1, 0.2], [2, 2, 2]);

snapshots = simulation.drawSnapshots("nSnapShots", 3);
snapshotCleanup = onCleanup(@() closeIfValid(snapshots));
verifyTrue(testCase, isgraphics(snapshots, "figure"));

animation = figure("Visible", "off");
animationCleanup = onCleanup(@() closeIfValid(animation));
simulation.animateSimResults("figure", animation, "frameRate", 20);
verifyTrue(testCase, isgraphics(animation, "figure"));
end

%% Shared physical fixtures and assertion utilities

function beam = makeBeam(nSegments, isCantilever, payloadMass)
% Straight, homogeneous 1.25 m beam along +z with rotational strain DoFs;
% shear and extension are constrained. Its per-length mass and spatial
% inertia provide the analytic reference values for the assembly tests.
% The base is fixed for a cantilever, or a y-axis revolute joint otherwise.
beam = elara.FlexibleLink;
beam.parentLink = 0;
beam.isCantilever = isCantilever;
beam.jointAxis = [0; 1; 0; 0; 0; 0];
beam.nSegments = nSegments;
beam.L = 1.25;
beam.Ba = [eye(3); zeros(3)];
beam.Bc = [zeros(3); eye(3)];
beam.xiRef = repmat([0; 0; 0; 0; 0; 1], 1, nSegments);
parameters = struct('E', 1e6, 'nu', 0.3, 'rho', 200, 'A', 1e-3, ...
    'I_x', 1e-5, 'I_y', 2e-5, 'J_P', 3e-5);
beam.beamParameters = beam.beamParameters.computeParameters(parameters);
if payloadMass > 0
    % Add one rigid payload at the last physical node, with no frame offset.
    % Its separately specified inertia must not inherit distributed weights.
    beam.g_a = repmat(eye(4), 1, 1, nSegments+1);
    beam.m_a = zeros(1, nSegments+1);
    beam.M_a = zeros(6, 6, nSegments+1);
    beam.m_a(end) = payloadMass;
    beam.M_a(:,:,end) = blkdiag(1e-4*eye(3), payloadMass*eye(3));
end
end

function simulation = makeSimulation
% Shared rigid planar pendulum: two 1 m, 1 kg links with actuated revolute
% joints and no joint damping. Start displaced from equilibrium and at rest.
% Four intervals of h=0.05 s, one Broyden iteration, and 1e-20 tolerances
% deliberately make ordinary gravity-driven motion fail at the first solve.
% Individual tests change gravity, torques, or solver settings to control
% when failure occurs or obtain successful runs.
links = systemDef_planarNLinkPendulum("nLinks", 2, "d", 0);
simulation = elara.Simulation(links, "displayInfo", false);
simulation.parameters.q0 = [pi/3; pi/4];
simulation.parameters.qDot0 = zeros(2,1);
simulation.parameters.tEnd = 0.2;
simulation.integrator.h = 0.05;
simulation.integrator.maxIterations = 1;
simulation.integrator.tolerance = 1e-20;
simulation.integrator.toleranceLimit = 1e-20;
simulation.integrator.showConsoleOutput = false;
end

function verifyIntegrationFailure(testCase, simulation, expectedStep)
% Run the public simulation path and check the common failure contract.
% A failed interval k leaves k accepted nodes, ending at (k-1)*h, while its
% attempted endpoint is discarded. Check warning/status, finite retained
% poses, and NaN final interval velocities; restore warning state afterward.
state = warning('query', 'elara:IntegrationFailed');
warningCleanup = onCleanup(@() warning(state));
[previousMessage, previousId] = lastwarn;
lastwarnCleanup = onCleanup(@() lastwarn(previousMessage, previousId));
warning('on', 'elara:IntegrationFailed');
lastwarn('');
% Capture printed output while leaving the warning available through lastwarn.
evalc('simulation = simulation.simulateSystem;');
[message, identifier] = lastwarn;
verifyEqual(testCase, identifier, 'elara:IntegrationFailed');
verifyNotEmpty(testCase, message);
verifyEqual(testCase, simulation.results.simulationStatus, 0);
verifyEqual(testCase, simulation.results.tout(end), ...
    (expectedStep-1)*simulation.integrator.h);
verifySize(testCase, simulation.results.q, [2, expectedStep]);
verifyTrue(testCase, all(isfinite(simulation.results.q), 'all'));
verifyTrue(testCase, all(isfinite(simulation.results.g), 'all'));
verifyTrue(testCase, all(isnan(simulation.results.q_dot(:,end))));
verifyTrue(testCase, all(isnan(simulation.results.eta(:,:,end)), 'all'));
end

function results = integrateRaw(simulation)
% Copy the public VI settings into the kernel's config, matching the wrapper.
% Call the available MATLAB/MEX kernel directly so these tests can distinguish
% raw trajectory truncation from the wrapper's status and warning handling.
config = elara.internal.integration.VIBroydenConfig;
config.h = simulation.integrator.h;
config.tolerance = simulation.integrator.tolerance;
config.toleranceLimit = simulation.integrator.toleranceLimit;
config.maxIterations = simulation.integrator.maxIterations;
config.JacobianIterationThreshold = simulation.integrator.JacobianIterationThreshold;
config.aTrapez = double(~simulation.integrator.useFirstOrderDissipation)/2;
if elara.internal.isMexAvailable('elara.mex.integrateVIBroyden_mex')
    results = elara.mex.integrateVIBroyden_mex( ...
        simulation.system, simulation.parameters, config);
else
    results = elara.internal.integration.integrateVIBroyden( ...
        simulation.system, simulation.parameters, config);
end
end

function assignSetting(obj, propertyName, value)
% Wrap assignment in a callable so verifyError can check validator failures.
obj.(propertyName) = value;
end

function simulation = simulateIgnoringFailureWarning(simulation)
% Suppress only the expected failure warning in status-focused tests, then
% restore its previous setting even if the simulation raises an exception.
state = warning('off', 'elara:IntegrationFailed');
warningCleanup = onCleanup(@() warning(state));
simulation = simulation.simulateSystem;
end

function closeIfValid(figureHandle)
% Cleanup may encounter an already-closed figure; close only live handles.
if isgraphics(figureHandle)
    close(figureHandle);
end
end
