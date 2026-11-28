function tests = SymbolicDynamicsRegressionTests
% Regressions for branched mass matrices and symbolic ODE interfaces.
tests = functiontests(localfunctions);
end

function setupOnce(~)
addpath(fullfile(elara.internal.getToolboxRootFolder, ...
    "examples", "example-systems"));
end

function testBranchedMassMatrixMatchesNumericSystem(testCase)
% A rigid fork has scalar zero coupling blocks; a flexible child also
% requires rectangular zero blocks between coordinates on separate branches.
for flexibleChild = [false, true]
    links = makeFork(flexibleChild);
    numericSystem = elara.SystemNum(links);
    symbolicSystem = elara.SystemSym(links);
    q = 0.1*(1:numericSystem.nDoF).';
    blocks = symbolicSystem.computeMassMatrix(q);
    actual = cell2mat(blocks);
    verifySize(testCase, actual, [numericSystem.nDoF, numericSystem.nDoF]);
    verifyEqual(testCase, actual, numericSystem.computeMassMatrix(q), ...
        'AbsTol', 1e-12);
end
end

function testSymbolicODEHelpersMatchNumericEquations(testCase)
% Compare independent numeric and symbolic mass-matrix/explicit equations.
% The explicit symbolic input must work while simPars.uConst remains empty.
definitions = {systemDef_planarNLinkPendulum("nLinks", 1), ...
    systemDef_rigid_robot, makeFork(false), makeFork(true)};
for iSystem = 1:numel(definitions)
    numericSystem = elara.SystemNum(definitions{iSystem});
    symbolicSystem = elara.SystemSym(definitions{iSystem});
    n = numericSystem.nDoF;
    m = numericSystem.nInputs;
    x = 0.1*sin((1:2*n).');
    u = (1:m).'/m;
    parameters = elara.SimulationParameters;
    numericParameters = parameters;
    numericParameters.uConst = u;
    expectedMass = elara.dynamics.num.firstOrderMassMatrix(0, x, numericSystem);
    expectedRHS = elara.dynamics.num.firstOrderRHS( ...
        0, x, numericSystem, numericParameters);
    expectedDerivative = elara.dynamics.num.firstOrderDerivative( ...
        0, x, numericSystem, numericParameters);

    for type = ["double", "DM", "MX", "SX"]
        switch type
            case "double"
                xArgument = x;
                uArgument = u;
            case "DM"
                xArgument = casadi.DM(x);
                uArgument = casadi.DM(u);
            case "MX"
                xArgument = casadi.MX.sym('x', 2*n, 1);
                uArgument = casadi.MX.sym('u', m, 1);
            case "SX"
                xArgument = casadi.SX.sym('x', 2*n, 1);
                uArgument = casadi.SX.sym('u', m, 1);
        end
        mass = elara.dynamics.sym.firstOrderMassMatrix(0, xArgument, symbolicSystem);
        rhs = elara.dynamics.sym.firstOrderRHS( ...
            0, xArgument, symbolicSystem, parameters, uArgument);
        derivative = elara.dynamics.sym.firstOrderDerivative( ...
            0, xArgument, uArgument, symbolicSystem, parameters);
        if type == "MX" || type == "SX"
            % Expansion is also required by OCP solvers using expand=true.
            evaluate = casadi.Function('ode_helpers', ...
                {xArgument, uArgument}, {mass, rhs, derivative}).expand();
            [mass, rhs, derivative] = evaluate(x, u);
        end
        verifyEqual(testCase, full(mass), expectedMass, 'AbsTol', 1e-12);
        verifyEqual(testCase, full(rhs), expectedRHS, ...
            'AbsTol', 1e-10, 'RelTol', 1e-9);
        verifyEqual(testCase, full(derivative), expectedDerivative, ...
            'AbsTol', 1e-9, 'RelTol', 1e-8);
    end
end
end

function testBranchedOCPSolvesWithEveryDiscretization(testCase)
% With gravity off, the zero trajectory is an exact equilibrium and has
% globally minimal control effort. It must remain feasible on a fork.
discretizations = {elara.ocp.DiscretizationVI, ...
    elara.ocp.DiscretizationRK("RK2"), elara.ocp.DiscretizationRK("RK4"), ...
    elara.ocp.DiscretizationImplicitMidpoint};
for iMethod = 1:numel(discretizations)
    problem = makeProblem;
    problem.discretization = discretizations{iMethod};
    problem = problem.initSolver;
    nStates = problem.systemNum.nDoF;
    if problem.discretization.type == "ode"
        nStates = 2*nStates;
    end
    [states, inputs, ~, ~, stats] = problem.solve( ...
        zeros(nStates, problem.nSteps+1), ...
        zeros(problem.systemNum.nInputs, problem.nSteps+1));
    verifyTrue(testCase, stats.success, stats.return_status);
    verifyEqual(testCase, states, zeros(size(states)), 'AbsTol', 1e-8);
    verifyEqual(testCase, inputs, zeros(size(inputs)), 'AbsTol', 1e-8);
end
end

function testShortOCPGridsReportSampleRequirements(testCase)
% Objective-component functions still evaluate derivatives when their costs
% are inactive. Reject insufficient grids clearly for direct and spline inputs.
for order = [2, 4]
    for useSplines = [false, true]
        problem = makeProblem;
        problem.finiteDifferenceOrder = order;
        problem.useSplineInputs = useSplines;
        problem.nInputSplinePoints = 4;
        problem.runningCostActive = false(5,1);
        minimumNodes = order + 2;
        problem.tEnd = problem.h*(minimumNodes-2);
        verifyError(testCase, @() problem.initSolver, 'elara:InsufficientSamples');

        % The exact minimum must still construct usable objective functions.
        problem.tEnd = problem.h*(minimumNodes-1);
        problem = problem.initSolver;
        nInputValues = problem.nSteps + 1;
        if useSplines
            nInputValues = problem.nInputSplinePoints;
        end
        objective = problem.evaluateObjectiveComponents( ...
            zeros(problem.systemNum.nDoF, problem.nSteps+1), ...
            zeros(problem.systemNum.nInputs, nInputValues));
        verifyEqual(testCase, objective, 0, 'AbsTol', 1e-12);
    end
end
end

function problem = makeProblem
problem = elara.ocp.Problem(makeFork(false));
problem.h = 0.25;
problem.tEnd = 0.75;
problem.q0 = zeros(problem.systemNum.nDoF,1);
problem.qDot0 = zeros(problem.systemNum.nDoF,1);
problem.qDotF = zeros(problem.systemNum.nDoF,1);
problem.simPars.g = 0;
problem.runningCostWeights = [1; 0; 0; 0; 0];
problem.runningCostActive = logical(problem.runningCostWeights);
problem.finalCostWeights = zeros(3,1);
problem.finalCostActive = false(3,1);
problem.nlpOptions = struct('print_time', false, ...
    'ipopt', struct('print_level', 0));
end

function links = makeFork(flexibleChild)
links = systemDef_planarNLinkPendulum("nLinks", 3, "d", 0.1);
links(3).parentLink = 1;
links(3).g_ref = elara.SE3.matrix(eye(3), [0.4; 0; 1]);
if flexibleChild
    beam = elara.FlexibleLink;
    beam.parentLink = 1;
    beam.jointAxis = [0; 1; 0; 0; 0; 0];
    beam.g_ref = links(3).g_ref;
    beam.L = 0.6;
    beam.nSegments = 2;
    beam.Ba = [eye(3); zeros(3)];
    beam.Bc = [zeros(3); eye(3)];
    beam.xiRef = repmat([0; 0; 0; 0; 0; 1], 1, beam.nSegments);
    links = [links(1:2); beam];
end
links(end).hasTCP = true;
end
