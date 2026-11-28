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

%% Supported beam selection matrices and constitutive inputs

function testSignedAndMixedSelectionMatricesAreRejected(testCase)
% Use a two-segment cantilever with healthy diagonal material stiffness.
% Sign flips and the exact orthogonal Hadamard block pass the old basis
% checks, but are not selections of the physical deformation components.
beam = makeBeam(2, true, 0);
signedBa = beam;
signedBa.Ba(:,1) = -signedBa.Ba(:,1);
signedBc = beam;
signedBc.Bc(:,1) = -signedBc.Bc(:,1);
H = blkdiag([1,1,1,1; 1,-1,1,-1; 1,1,-1,-1; 1,-1,-1,1]/2, eye(2));
mixed = beam;
mixed.Ba = H(:,1:3);
mixed.Bc = H(:,4:6);
for candidate = {signedBa, signedBc, mixed}
    verifyError(testCase, @() elara.SystemNum(candidate{1}), ...
        'elara:InvalidSelectionMatrix');
end
end

function testSelectionMatricesAllowReorderedAxes(testCase)
% Reordering the selected rotational axes is supported. On the cantilever,
% stiffness and damping must follow that coordinate order for every segment.
beam = makeBeam(2, true, 0);
order = [3; 1; 2];
beam.Ba = beam.Ba(:,order);
beam.Bc = beam.Bc(:,order);
beam.beamParameters.d = (1:6).';
system = elara.SystemNum(beam);
l = beam.L/beam.nSegments;
stiffness = diag(beam.beamParameters.Cgen);
verifyEqual(testCase, system.cSys, repmat(l*stiffness(order), beam.nSegments, 1), ...
    'AbsTol', 1e-12);
verifyEqual(testCase, system.dSys, ...
    repmat(l*beam.beamParameters.d(order), beam.nSegments, 1), 'AbsTol', 1e-12);
end

function testCoupledBeamStiffnessIsRejected(testCase)
% Add symmetric coupling between bending modes of the same cantilever.
% A stiffness vector cannot represent it, so assembly must reject the input
% rather than silently discarding the material matrix's off-diagonal terms.
beam = makeBeam(2, true, 0);
C = beam.beamParameters.Cgen;
C(1,2) = sqrt(C(1,1)*C(2,2))/4;
C(2,1) = C(1,2);
beam.beamParameters.Cgen = C;
verifyError(testCase, @() elara.SystemNum(beam), 'elara:UnsupportedBeamStiffness');
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

function testSmallerToleranceLimitIsRaisedBeforeIntegration(testCase)
% A loose target used with the fixture's very strict hard limit must warn
% and use the target as the effective limit for integration and assessment.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.integrator.tolerance = 1;
originalLimit = simulation.integrator.toleranceLimit;
state = warning('on', 'elara:ToleranceLimitAdjusted');
testCase.addTeardown(@() warning(state));
[previousMessage, previousId] = lastwarn;
testCase.addTeardown(@() lastwarn(previousMessage, previousId));
lastwarn('');
evalc('simulation = simulation.simulateSystem;');
[~, identifier] = lastwarn;
verifyEqual(testCase, identifier, 'elara:ToleranceLimitAdjusted');
verifyEqual(testCase, numel(simulation.results.tout), 2);
verifyEqual(testCase, simulation.results.solverExitFlag(1), 0);
verifyGreaterThan(testCase, simulation.results.solverResidual(1), ...
    originalLimit);
verifyLessThanOrEqual(testCase, simulation.results.solverResidual(1), ...
    simulation.integrator.tolerance);
verifyEqual(testCase, simulation.results.simulationStatus, 1);
end

function testConsistentToleranceLimitsDoNotWarn(testCase)
% Equal and larger hard limits require no adjustment. An equilibrium run
% isolates warning behavior from ordinary integration failure warnings.
simulation = makeSimulation;
simulation.parameters.tEnd = simulation.integrator.h;
simulation.parameters.g = 0;
simulation.parameters.q0 = zeros(2,1);
simulation.integrator.maxIterations = 0;
state = warning('error', 'elara:ToleranceLimitAdjusted');
testCase.addTeardown(@() warning(state));
for limit = simulation.integrator.tolerance * [1, 2]
    simulation.integrator.toleranceLimit = limit;
    simulation = simulation.simulateSystem;
    verifyEqual(testCase, simulation.results.simulationStatus, 1);
end
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

function testODERejectsInvalidConstantInputs(testCase)
% A one-input pendulum must reject two supplied torques before integration,
% including when an existing MEX kernel would silently ignore the vector.
for interface = ["object", "function"]
    for useMassMatrix = [false, true]
        simulation = makeODEInputSimulation(interface, useMassMatrix);
        simulation.parameters.uConst = [100; 200];
        verifyError(testCase, @() simulation.simulateSystem, ...
            'elara:InvalidConstantInputDimensions');
    end
end
end

function testODERejectsInvalidSampledInputRows(testCase)
% Every active sampled input needs one row per actuator. A wrong channel
% count previously disabled the supplied trajectory without a diagnostic.
for interface = ["object", "function"]
    for useMassMatrix = [false, true]
        simulation = makeODEInputSimulation(interface, useMassMatrix);
        simulation.parameters.uSampleTimes = [0; 0.1; 0.2];
        simulation.parameters.uSampleValues = ones(2,3);
        verifyError(testCase, @() simulation.simulateSystem, ...
            'elara:InvalidSampledInputDimensions');
    end
end
end

function testODERejectsMismatchedInputSampleCounts(testCase)
% A trajectory with missing sample values must fail instead of producing
% the unforced motion. Exercise both interfaces and both equation forms.
for interface = ["object", "function"]
    for useMassMatrix = [false, true]
        simulation = makeODEInputSimulation(interface, useMassMatrix);
        simulation.parameters.uSampleTimes = [0; 0.1; 0.2];
        simulation.parameters.uSampleValues = [1, 2];
        verifyError(testCase, @() simulation.simulateSystem, ...
            'elara:InvalidInputSampleCount');
    end
end
end

function testODEInputsMatchAnalyticMotion(testCase)
% With gravity and damping off, the one-link pendulum obeys I*q_ddot=u(t).
% Constant plus linearly sampled torque therefore gives a cubic q(t).
% Also preserve the documented behavior that either empty sampled-input
% array disables that contribution, even if the other array is populated.
% Columns: constant input, sample times, sample values, total u(0), du/dt.
cases = {
    [],  [],             [],                  0,   0
    0.3, [],             [],                  0.3, 0
    [],  [0; 0.1; 0.2],  [0.2, 0.35, 0.5],    0.2, 1.5
    0.3, [0; 0.1; 0.2],  [0.2, 0.35, 0.5],    0.5, 1.5
    0.3, [],             [1, 2; 3, 4],        0.3, 0
    0.3, [0; 0.1; 0.2],  [],                  0.3, 0
    };
for interface = ["object", "function"]
    for useMassMatrix = [false, true]
        for k = 1:size(cases,1)
            simulation = makeODEInputSimulation(interface, useMassMatrix);
            simulation.parameters.uConst = cases{k,1};
            simulation.parameters.uSampleTimes = cases{k,2};
            simulation.parameters.uSampleValues = cases{k,3};
            link = simulation.links;
            inertia = link.J(2,2) + link.m*link.g_J_B(1,4)^2;
            simulation = simulation.simulateSystem;
            t = simulation.results.tout.';
            q0 = simulation.parameters.q0;
            qDot0 = simulation.parameters.qDot0;
            expectedQ = q0 + qDot0*t + cases{k,4}/(2*inertia)*t.^2 ...
                + cases{k,5}/(6*inertia)*t.^3;
            expectedQDot = qDot0 + cases{k,4}/inertia*t ...
                + cases{k,5}/(2*inertia)*t.^2;
            verifyEqual(testCase, simulation.results.simulationStatus, 1);
            verifyEqual(testCase, t(end), simulation.parameters.tEnd, 'AbsTol', 1e-12);
            verifyEqual(testCase, simulation.results.q, expectedQ, 'AbsTol', 5e-8);
            verifyEqual(testCase, simulation.results.q_dot, expectedQDot, 'AbsTol', 5e-8);
        end
    end
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

function testFiniteDifferenceVelocitiesOnUniformGrids(testCase)
% Quadratic and linear coordinates have exact finite-difference velocities
% for both supported orders. Cover ordinary, tiny, and large time scales.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 2));
s = linspace(0, 1, 9);
q = [s.^2; 0.2+3*s];
for scale = [0.8, 8e-10, 8e8]
    times = scale*s.';
    expected = [2*s; 3*ones(size(s))]/scale;
    for order = [2, 4]
        results = elara.SimulationResults.fromStateTrajectory(system, times, q, ...
            "finiteDifferenceOrder", order);
        verifyEqual(testCase, results.q_dot, expected, ...
            'AbsTol', 1e-10/scale, 'RelTol', 1e-10);
    end
end
end

function testFiniteDifferenceMinimumSampleCounts(testCase)
% Stencil boundaries must raise a useful error instead of indexing beyond
% the trajectory. At the exact minimum, quadratic derivatives remain exact.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 1));
functions = {@diff2ndOrder, @diff4thOrder};
orders = [2, 4];
for iOrder = 1:numel(orders)
    order = orders(iOrder);
    minimumSamples = order + 2;
    for nTimes = 2:minimumSamples-1
        times = linspace(0, 1, nTimes).';
        q = times.'.^2;
        h = times(2)-times(1);
        verifyError(testCase, @() elara.SimulationResults.fromStateTrajectory( ...
            system, times, q, "finiteDifferenceOrder", order), ...
            'elara:InsufficientSamples');
        verifyError(testCase, @() functions{iOrder}(q, h), ...
            'elara:InsufficientSamples');
        verifyError(testCase, @() diffHigherOrder(q, h, order), ...
            'elara:InsufficientSamples');
    end
    times = linspace(0, 1, minimumSamples).';
    q = times.'.^2;
    results = elara.SimulationResults.fromStateTrajectory( ...
        system, times, q, "finiteDifferenceOrder", order);
    verifyEqual(testCase, results.q_dot, 2*times.', 'AbsTol', 1e-12);
    [qDot, qDDot] = diffHigherOrder(q, times(2)-times(1), order);
    verifyEqual(testCase, qDot, 2*times.', 'AbsTol', 1e-12);
    verifyEqual(testCase, qDDot, 2*ones(size(q)), 'AbsTol', 1e-12);
end
verifyError(testCase, @() diffHigherOrder([0, 1], 1, 1), ...
    'elara:InsufficientSamples');

% Explicit velocities remain valid for a two-sample trajectory.
results = elara.SimulationResults.fromStateTrajectory(system, [0; 1], [0, 1], [0, 2]);
verifyEqual(testCase, results.q_dot, [0, 2]);
end

function testFiniteDifferenceGridTolerance(testCase)
% Small timestamp perturbations are accepted, while a change large enough
% to exceed the relative spacing tolerance is rejected for both orders.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 1));
for order = [2, 4]
    times = (0:0.1:0.5).';
    times(3) = times(3) + 1e-11;
    results = elara.SimulationResults.fromStateTrajectory(system, times, times.'.^2, ...
        "finiteDifferenceOrder", order);
    verifyEqual(testCase, results.q_dot, 2*times.', 'AbsTol', 1e-7);

    times(3) = times(3) + 1e-7;
    verifyError(testCase, @() elara.SimulationResults.fromStateTrajectory( ...
        system, times, times.'.^2, "finiteDifferenceOrder", order), ...
        'elara:NonuniformTimeGrid');
end
end

function testFiniteDifferenceGridWithLargeTimeOrigin(testCase)
% Subtraction of large timestamps introduces more roundoff than a check
% based only on relative step size would allow. The nominal grid is uniform.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 1));
localTimes = (0:0.1:0.5).';
for origin = [-1e8, 1e8]
    times = origin + localTimes;
    for order = [2, 4]
        results = elara.SimulationResults.fromStateTrajectory( ...
            system, times, localTimes.', "finiteDifferenceOrder", order);
        verifyEqual(testCase, results.q_dot, ones(1,numel(times)), 'AbsTol', 1e-12);
    end
end
end

function testFiniteDifferenceRejectsNonuniformTimes(testCase)
% The original q=t^2 counterexample must fail before returning an incorrect
% velocity. Very small times must not be hidden by a fixed absolute tolerance.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 1));
baseTimes = [0; 0.1; 0.3; 0.6; 1; 1.4];
for scale = [1, 1e-12, 1e12]
    times = scale*baseTimes;
    for order = [2, 4]
        verifyError(testCase, @() elara.SimulationResults.fromStateTrajectory( ...
            system, times, baseTimes.'.^2, "finiteDifferenceOrder", order), ...
            'elara:NonuniformTimeGrid');
    end
end
end

function testNonuniformTimesWithExplicitVelocities(testCase)
% Supplied velocities bypass finite differences. Preserve finite values and
% explicitly supplied NaNs, including an array of entirely unknown velocities.
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 2));
times = [0; 0.1; 0.3; 0.6; 1; 1.4];
q = [times.'.^2; times.'.^3];
qDot = [2*times.'; 3*times.'.^2];
for velocities = {qDot, [qDot(:,1:end-1), nan(2,1)], nan(size(q))}
    results = elara.SimulationResults.fromStateTrajectory(system, times, q, velocities{1});
    verifyEqual(testCase, results.tout, times);
    verifyEqual(testCase, results.q, q);
    verifyTrue(testCase, isequaln(results.q_dot, velocities{1}));
end
end

function testInvalidTimeGridsForFiniteDifferences(testCase)
system = elara.SystemNum(systemDef_planarNLinkPendulum("nLinks", 1));
grids = {[], 0, [0; 0; 0.1; 0.2], [0.3; 0.2; 0.1; 0], ...
    [0; 0.1; NaN; 0.3], [0; 0.1; 0.2; Inf], 1i*(0:0.1:0.3).'};
for k = 1:numel(grids)
    times = grids{k};
    verifyError(testCase, @() elara.SimulationResults.fromStateTrajectory( ...
        system, times, zeros(1,numel(times))), 'elara:InvalidTimeGrid');
end
end

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

function simulation = makeODEInputSimulation(interface, useMassMatrix)
% Single actuated link with constant joint inertia and no gravitational or
% dissipative forces, so prescribed torque has a closed-form trajectory.
links = systemDef_planarNLinkPendulum("nLinks", 1, "d", 0);
simulation = elara.Simulation(links, "displayInfo", false);
simulation.parameters.g = 0;
simulation.parameters.q0 = 0.2;
simulation.parameters.qDot0 = 0.3;
simulation.parameters.tEnd = 0.2;
if interface == "object"
    simulation.integrator = elara.integration.ODEDirect;
    simulation.integrator.odeObject.Solver = "ode15s";
    simulation.integrator.odeObject.AbsoluteTolerance = 1e-10;
    simulation.integrator.odeObject.RelativeTolerance = 1e-10;
else
    simulation.integrator = elara.integration.ODEDirectFunctionBased;
    simulation.integrator.solverOptions = odeset('AbsTol', 1e-10, 'RelTol', 1e-10);
end
simulation.integrator.useMassMatrixForm = useMassMatrix;
simulation.integrator.showConsoleOutput = false;
end

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
