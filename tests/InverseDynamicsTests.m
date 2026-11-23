% Validation/Unit Tests for Numerical Inverse Dynamics Functions
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "example-systems"));

% Make randomized test inputs reproducible
rng default;

%%% Script Parameters
MAKE_PLOTS = false;
tEnd = 1;

% Use random time step
h = 0.075 + rand(1)*0.05;

%% Rigid Robot
% fully actuated, unconstrained
testInvDynSystem( ...
    systemDef_rigid_robot(), tEnd, h, -inf, inf, ...
    "makePlots", MAKE_PLOTS);

%% Planar Pendulum
% fully actuated, unconstrained
testInvDynSystem( ...
    systemDef_planarNLinkPendulum("nLinks", 5), tEnd, h, -inf, inf, ...
    "makePlots", MAKE_PLOTS);

%% Continuum Manipulator
% under-actuated, with control limits
testInvDynSystem( ...
    systemDef_continuum_manipulator(), tEnd, h, -1, 1, ...
    "makePlots", MAKE_PLOTS);

%%% Local Function
function testInvDynSystem(links,tEnd,h, lbu, ubu, opts)
    arguments
        links   (:,1) elara.abstract.Link
        tEnd    (1,1) double {mustBePositive}
        h       (1,1) double {mustBePositive}

        % Scalar lower and upper bounds (applied to all inputs)
        lbu    (1,1) double
        ubu    (1,1) double

        opts.makePlots (1,1) logical = false;
    end

    %%% Define system

    sim = elara.Simulation(links, "displayInfo", false);

    % Check if system is underactuated
    isUnderactuated = sim.system.nDoF > sim.system.nInputs;

    sim.parameters.tEnd = tEnd;
    tout = 0:h:tEnd;

    lbuVec = ones(sim.system.nInputs,1)*lbu;
    ubuVec = ones(sim.system.nInputs,1)*ubu;

    %%% Generate desired trajectory (cosine functions)

    a1 = rand(sim.system.nDoF,1)*5;
    a2 = rand(sim.system.nDoF,1)-0.5;
    a3 = rand(sim.system.nDoF,1)-0.5;

    phase = a1.*tout + a3;
    q   = a2.*cos(phase);
    qd  = -a1.*a2.*sin(phase);
    qdd = -(a1.^2).*a2.*cos(phase);

    % Plot desired trajectory
    if opts.makePlots
        figure("Name", "Desired Trajectory");
        tiledlayout("vertical");
        nexttile;
        plot(tout, q);
        grid on
        title("Coordinates");
        nexttile;
        plot(tout, qd);
        grid on
        title("Velocities");
        nexttile;
        plot(tout, qdd);
        grid on
        title("Accelerations");
    end

    %%% Compute Inverse Dynamics and Residual for DEL Functions

    % Compute input trajectory via inverse dynamics
    [u_DEL, solInfo_DEL] = elara.dynamics.num.inverseDynamicsDEL( ...
        sim.system, sim.parameters, q, qd, h, lbuVec, ubuVec);


    % Compute DEL residuals
    a = 0.5;

    nSteps = size(q, 2) - 1;
    res_DEL = nan(sim.system.nDoF, nSteps+1);

    % Initial step
    [g_0, g_rel_0] = sim.system.computeFwdKin(q(:,1));
    [~  , g_rel_1] = sim.system.computeFwdKin(q(:,2));
    eta_0 = sim.system.computeDiscreteAbsoluteVelocitiesFast(g_rel_0,g_rel_1, h);
    res_DEL(:,1) = elara.dynamics.num.DELResidualInitialStep_noKinematics( ...
        sim.system, h, sim.parameters, q(:,1), q(:,2), g_0, g_rel_0, eta_0, ...
        u_DEL(:,1), qd(:,1), a);

    % Intermediate steps
    for k = 2:nSteps
        res_DEL(:,k) = elara.dynamics.num.DELResidual( ...
            sim.system, sim.parameters, q(:,k-1), q(:,k), q(:,k+1), ...
            u_DEL(:,k), h, a);
    end

    % Final step
    res_DEL(:,nSteps+1) = elara.dynamics.num.DELResidualFinalStep( ...
        sim.system, h, sim.parameters, q(:,nSteps), q(:,nSteps+1), ...
        u_DEL(:,nSteps+1), qd(:,nSteps+1), a);

    %%% Compute Inverse Dynamics and Residual for ODE Functions

    % Compute input trajectory via inverse dynamics
    [u_ODE, solInfo_ODE] = elara.dynamics.num.inverseDynamicsODE( ...
        sim.system, sim.parameters, q, qd, qdd, lbuVec, ubuVec);

    % Compute ODE residuals
    res_ODE = nan(sim.system.nDoF, nSteps+1);
    for k = 1:nSteps+1
        res_ODE(:,k) = elara.dynamics.num.secondOrderODEResidual( ...
            tout(k), q(:,k), qd(:,k), qdd(:,k), u_ODE(:,k), ...
            sim.system, sim.parameters);
    end


    %%% Plot Data

    if opts.makePlots
        % Plot inputs
        figure("Name", "ID Input Trajectories");
        plot(tout, u_DEL, "-o");
        hold on;
        plot(tout, u_ODE, "--x");
        colororder(lines(sim.system.nInputs));
        grid on;
        legend([ ...
            arrayfun(@(x) sprintf("DEL u_{%d}", x), 1:sim.system.nInputs), ...
            arrayfun(@(x) sprintf("ODE u_{%d}", x), 1:sim.system.nInputs), ...
            ]);

        % Plot residuals
        figure("Name", "ODE Residuals");
        semilogy(tout, vecnorm(res_DEL), "-o", "DisplayName", "DEL");
        hold on;
        semilogy(tout, vecnorm(res_ODE), "-o", "DisplayName", "ODE");
        grid on;
        legend();
    end

    %%% Check Residuals

    if isUnderactuated
        % Under-actuated case for an arbitrary, possibly un-feasible
        % desired trajectory

        %%% Check if overall residuals are within inverse dynamics solver residuals
        % Compute scaled squared residual that corresponds to the given
        % lsqlinsquare residual
        scale = h*ones(1, size(res_DEL, 2));
        scale(1) = h*(1-a);
        scale(end) = h*a;
        reg = 5e-4; % Regularization factor from solveSystemInputs()

        J_DEL = sum((res_DEL./scale).^2, 1) + (reg*sum(u_DEL, 1)).^2;
        J_ODE = sum((res_ODE).^2, 1) + (reg*sum(u_ODE, 1)).^2;

        assert(all(J_DEL - eps < solInfo_DEL.resNorm.'), ...
            "DEL residuals not within tolerances.")
        assert(all(J_ODE - eps < solInfo_ODE.resNorm.'), ...
            "ODE residuals not within tolerances.")

        %%% Check if computed inputs are within bounds
        tolU = 1e-10;
        assert(all(u_DEL(:) > lbu - tolU) && all(u_DEL(:) < ubu + tolU), ...
            "Computed DEL inputs not within specified bounds.")
        assert(all(u_ODE(:) > lbu - tolU) && all(u_ODE(:) < ubu + tolU), ...
            "Computed ODE inputs not within specified bounds.")

    else
        % Fully actuated case: Residuals should be at machine precision
        assert(norm(res_DEL) < 1e-12, ...
            "DEL residuals not within tolerances.")
        assert(norm(res_ODE) < 1e-12, ...
            "ODE residuals not within tolerances.")
    end
end