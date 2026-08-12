% Validate the CasADi DEL functions by comparing them to the numerical functions
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "example-systems"));

% Absolute test tolerances
tolKinematics = 1e-12;
tolStatics = 1e-10;
tolVelocities = 1e-8;
tolDEL = 1e-8;
tolMassMatrix = 1e-12;
tolBoundaryDEL = 1e-10;

% The continuous-time RHS uses a scale-aware tolerance because the
% flexible systems have poorly conditioned mass matrices.
tolContinuousAbs = 1e-8;
tolContinuousRel = 1e-5;

% Make randomized test inputs reproducible
rng default;

% System definitions
systemNames = [
    "Rigid robot"
    "Continuum manipulator"
    "Rigid-flexible robot"
    "Planar manipulator"
    ];
systemDefinitions = {
    @systemDef_rigid_robot
    @systemDef_continuum_manipulator
    @systemDef_rigid_flexible_robot
    @() systemDef_planarNLinkPendulum("d", 0)
    };

% Discrete time step and quadrature factor
h = 2^-5;
a = 1/2;


%% Test kinematics

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);

    q = rand(systemNum.nDoF, 1);

    [gNum, gRelNum] = systemNum.computeFwdKin(q);
    JNum = systemNum.computeGeomJacobianFast(q, gRelNum);

    [gSym, gRelSym] = systemSym.computeFwdKin(q);
    JSymBlocks = systemSym.computeGeomJacobianFast(q, gRelSym);

    gRelSymMat = elara.SE3.element2Matrix(gRelSym);
    gSymMat = elara.SE3.element2Matrix(gSym);

    % Assemble the block-cell symbolic Jacobian in the same array format
    % as the numerical Jacobian.
    JSym = zeros(size(JNum));
    for iFrame = 1:systemSym.nFrames
        for iBlock = 1:systemSym.nFrames
            if ~isempty(JSymBlocks{iFrame, iBlock})
                qIndices = double(systemSym.frames.getQIndices(iBlock));
                JSym(:,qIndices,iFrame) = ...
                    full(JSymBlocks{iFrame, iBlock});
            end
        end
    end

    errorGRel = max(abs(gRelNum(:) - gRelSymMat(:)));
    errorG = max(abs(gNum(:) - gSymMat(:)));
    errorJ = max(abs(JNum(:) - JSym(:)));

    assert(errorGRel <= tolKinematics, sprintf( ...
        "%s: Relative kinematics error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorGRel, tolKinematics));
    assert(errorG <= tolKinematics, sprintf( ...
        "%s: Absolute kinematics error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorG, tolKinematics));
    assert(errorJ <= tolKinematics, sprintf( ...
        "%s: Geometric Jacobian error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorJ, tolKinematics));
end


%% Test statics

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);
    parameters = elara.SimulationParameters;

    q = rand(systemNum.nDoF, 1) * 2;
    u = rand(systemNum.nInputs, 1) * 20;

    residualNum = elara.statics.num.residual(systemNum, parameters, q, u);

    qSym = casadi.MX.sym('q', systemNum.nDoF, 1);
    uSym = casadi.MX.sym('u', systemNum.nInputs, 1);
    residualSym = elara.statics.sym.residual(systemSym, parameters, qSym, uSym);
    residualFunction = casadi.Function( ...
        'static_residual', {qSym, uSym}, {residualSym});
    residualSymNum = full(residualFunction(q, u));

    errorStatics = max(abs(residualSymNum(:) - residualNum(:)));
    assert(errorStatics <= tolStatics, sprintf( ...
        "%s: Static residual error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorStatics, tolStatics));
end


%% Test velocities

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);

    q_k = rand(systemNum.nDoF, 1) * 2;
    q_k1 = rand(systemNum.nDoF, 1) * 2;

    [~, g_rel_kNum] = systemNum.computeFwdKin(q_k);
    [~, g_rel_k1Num] = systemNum.computeFwdKin(q_k1);
    etaNum = systemNum.computeDiscreteAbsoluteVelocities( ...
        g_rel_kNum, g_rel_k1Num, h);

    [~, g_rel_kSym] = systemSym.computeFwdKin(q_k);
    [~, g_rel_k1Sym] = systemSym.computeFwdKin(q_k1);
    etaSym = systemSym.computeDiscreteAbsoluteVelocities( ...
        g_rel_kSym, g_rel_k1Sym, h);
    etaSymNum = cell2mat(etaSym.');

    errorVelocities = max(abs(etaSymNum(:) - etaNum(:)));
    assert(errorVelocities <= tolVelocities, sprintf( ...
        "%s: Velocity error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorVelocities, tolVelocities));
end


%% Test DEL functions

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);
    parameters = elara.SimulationParameters;

    q_k0 = rand(systemNum.nDoF, 1) * 2;
    q_k = rand(systemNum.nDoF, 1) * 2;
    q_k1 = rand(systemNum.nDoF, 1) * 2;
    u_k = rand(systemNum.nInputs, 1) * 20;

    residualNum = elara.dynamics.num.DELResidual( ...
        systemNum, parameters, q_k0, q_k, q_k1, u_k, h, a);

    % Direct evaluation through the symbolic system
    residualSymDirect = full(elara.dynamics.sym.DELResidual( ...
        systemSym, parameters, q_k0, q_k, q_k1, u_k, h, a));
    errorDELDirect = max(abs(residualNum(:) - residualSymDirect(:) / h));
    assert(errorDELDirect <= tolDEL, sprintf( ...
        "%s: Direct DEL residual error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorDELDirect, tolDEL));

    % Evaluation of the generated CasADi graph
    qSym_k0 = casadi.MX.sym('q_k0', systemNum.nDoF, 1);
    qSym_k = casadi.MX.sym('q_k', systemNum.nDoF, 1);
    qSym_k1 = casadi.MX.sym('q_k1', systemNum.nDoF, 1);
    uSym_k = casadi.MX.sym('u_k', systemNum.nInputs, 1);

    residualSym = elara.dynamics.sym.DELResidual( ...
        systemSym, parameters, qSym_k0, qSym_k, qSym_k1, uSym_k, h, a);
    residualFunction = casadi.Function('DEL_residual', ...
        {qSym_k0, qSym_k, qSym_k1, uSym_k}, {residualSym});
    residualSymGraph = full(residualFunction(q_k0, q_k, q_k1, u_k));

    errorDELGraph = max(abs(residualNum(:) - residualSymGraph(:) / h));
    assert(errorDELGraph <= tolDEL, sprintf( ...
        "%s: CasADi DEL residual error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorDELGraph, tolDEL));
end


%% Test mass matrix

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);

    q_k = rand(systemNum.nDoF, 1) * 2;

    massMatrixNum = systemNum.computeMassMatrix(q_k);
    massMatrixSym = cell2mat(systemSym.computeMassMatrix(q_k));

    errorMassMatrix = max(abs(massMatrixNum(:) - massMatrixSym(:)));
    assert(errorMassMatrix <= tolMassMatrix, sprintf( ...
        "%s: Mass matrix error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorMassMatrix, tolMassMatrix));
end


%% Test first and last step functions

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);
    parameters = elara.SimulationParameters;

    u_0 = rand(systemNum.nInputs, 1) * 20;
    q_0 = rand(systemNum.nDoF, 1) * 2;
    q_1 = rand(systemNum.nDoF, 1) * 2;
    parameters.qDot0 = rand(systemNum.nDoF, 1);
    qDotEnd = rand(systemNum.nDoF, 1);

    firstStepResidualNum = elara.dynamics.num.DELResidualInitialStep( ...
        systemNum, h, parameters, q_0, q_1, u_0, a);
    firstStepResidualSym = full(elara.dynamics.sym.DELResidualInitialStep( ...
        systemSym, parameters, q_0, q_1, u_0, parameters.qDot0, h, a));

    errorFirstStep = max(abs( ...
        firstStepResidualNum(:) - firstStepResidualSym(:)));
    assert(errorFirstStep <= tolBoundaryDEL, sprintf( ...
        "%s: First-step DEL error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorFirstStep, tolBoundaryDEL));

    lastStepResidualNum = elara.dynamics.num.DELResidualFinalStep( ...
        systemNum, h, parameters, q_0, q_1, u_0, qDotEnd, a);
    lastStepResidualSym = full(elara.dynamics.sym.DELResidualFinalStep( ...
        systemSym, parameters, q_0, q_1, u_0, qDotEnd, h, a));

    errorLastStep = max(abs( ...
        lastStepResidualNum(:) - lastStepResidualSym(:)));
    assert(errorLastStep <= tolBoundaryDEL, sprintf( ...
        "%s: Last-step DEL error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorLastStep, tolBoundaryDEL));
end


%% Test continuous-time EOM functions

for iSystem = 1:numel(systemDefinitions)
    links = systemDefinitions{iSystem}();
    simulation = elara.Simulation(links, "displayInfo", false);
    systemNum = simulation.system;
    systemSym = elara.SystemSym(links);
    parameters = elara.SimulationParameters;

    x = rand(2 * systemNum.nDoF, 1);
    u = rand(systemNum.nInputs, 1);
    parameters.uConst = u;

    rhsNum = elara.dynamics.num.firstOrderDerivative( ...
        0, x, systemNum, parameters);
    rhsSym = full(elara.dynamics.sym.firstOrderDerivative( ...
        0, casadi.DM(x), u, systemSym, parameters));

    errorContinuous = max(abs(rhsNum(:) - rhsSym(:)));
    scaleContinuous = max(norm(rhsNum, inf), norm(rhsSym, inf));
    toleranceContinuous = tolContinuousAbs + ...
        tolContinuousRel*scaleContinuous;
    assert(errorContinuous <= toleranceContinuous, sprintf( ...
        "%s: Continuous-time EOM error %.3e exceeds tolerance %.3e.", ...
        systemNames(iSystem), errorContinuous, toleranceContinuous));
end
