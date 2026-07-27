%% Optimal Control / Trajectory Generation for the Rigid Lab Robot
% to follow a prescribed TCP trajectory
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "exampleSystems"));

%% Script settings

% False = use zero initial guess
COMPUTE_IG = 1;

%% Define System

links = systemDef_rigid_robot("d", 0);
MBSim = elara.Simulation(links, "displayInfo", false);


%% Define OCP

OCP = OCPDefinition;
OCP.system = elara.SystemSym(links);

OCP.q0    = zeros(MBSim.system.nDoF,1); % Initial configuration
OCP.qDot0 = zeros(MBSim.system.nDoF,1); % Initial velocity
OCP.qDotF = zeros(MBSim.system.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = ones(MBSim.system.nInputs,1)*-100;
OCP.uMax = ones(MBSim.system.nInputs,1)*100;

% End time, sample time
OCP.h = 1e-2;
OCP.tF = 2;

% Desired TCP pose
OCP.x_TCP_F = [0.7; 0.2; 0.3];
OCP.R_TCP_F = []; % Rotation arbitrary

OCP.simPars = MBSim.parameters;

% Running cost
OCP.wRC = [ % weights
    1e-2/2 % Norm u
    0      % Norm u_dot
    0      % Norm u_ddot
    0      % Norm q_ddot
    1e3    % TCP error
    ];
OCP.iRC = logical(OCP.wRC); % Defines which cost terms are active

% No final time cost term
OCP.iFC = zeros(3,1); % Defines which cost terms are active

OCP.addTCPFinalTimeConstraint = false;
OCP.tPreAct  = 5*2^-5;
OCP.tPostAct = 2*2^-5;

OCP.qMin = ones(MBSim.system.nDoF, 1)*-2*pi;
OCP.qMax = ones(MBSim.system.nDoF, 1)*2*pi;

% Control trajectory parameterization with splines
OCP.useSplineInputs = true;
OCP.nInputSplinePoints = 20;
OCP.inputSplineOrder = 3;

% NLP object / solver options
OCP.nlpOpts.expand = false;


%% Visualize reference configuration and target position

[~, vis] = MBSim.visualizeSystemRefConf();
CoordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
drawWorkspace(OCP.workSpaceDef, "createFigure", false);


%% Generate desired TCP trajectory

OCP.x_TCP_traj = generateDesiredTCPTrajLinear(MBSim, OCP);


%% Initial Guess based on ODE Inverse Dynamics

[q_init_ode, qd_init_ode, u_init_ode] = OCPComputeInitialGuess_InvDyn( ...
    MBSim, OCP, "invDynMethod", "ODE", "createDebugPlots", false);

fh_IG_ode = plotOCPqu(OCP, q_init_ode, u_init_ode, "figureName", "Initial Guess ODE", "plotDerivatives", true);

% Compute Initial Guess
if COMPUTE_IG
    [q_init, qd_init, u_init, MBSimIG] = OCPComputeInitialGuess_InvDyn( ...
        MBSim, OCP, "createDebugPlots", false, "invDynMethod", "ODE");

    fh_IG = plotOCPqu(OCP, q_init, u_init, "figureName", "Initial Guess", "plotDerivatives", true);

    % Animate results
    fig = init3Dplot('Name', "Animation Initial Guess");
    CoordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
    drawWorkspace(OCP.workSpaceDef, "createFigure", false);
    MBSimIG.animateSimResults("figure", fig);
else
    q_init = repmat(OCP.q0, [1,OCP.nSteps+1]);
    u_init = repmat(OCP.u0, [1,OCP.nSteps+1]);
end


figure("Name", "Comp IG");
plot(OCP.tout, u_init_ode, '-o', "DisplayName", "ODE");
hold on;
plot(OCP.tout, u_init, '--x', "DisplayName", "VI");
legend;
grid on;
colororder(lines(3));

if OCP.useSplineInputs
    % Compute control points for initial guess
    B = OCP.getInputSplineBasisMatrix;
    u_init_z =  (B \ u_init.').';

    % Plot initial guess fit
    figure("Name", "Initial Guess B-Spline Fit");
    plot(OCP.tout, u_init, "-.x", "DisplayName", "Original Data");
    hold on;
    plot(OCP.tout, B*u_init_z.', "--o", "DisplayName", "Fitted Spline");

    grid on;
    colororder(lines(MBSim.system.nInputs));
    legend;
else
    u_init_z = u_init;
end


%% Define ODE OCP Solver

if 1
    OCP_ODE = OCP;
    OCP_ODE.discretization = OCPIntegratorRK("RK2");
    OCP_ODE.Name = "RK2";
    x_init = [q_init; qd_init];

    OCP_ODE = OCP_ODE.initSolver;
    OCP_ODE.plotConstraintResiduals(x_init, u_init_z, "figureName", "Constr. Res. IG");

    % Solve ODE OCP
    [x_sol, u_sol_z, sol, stats] = OCP_ODE.solve(x_init, u_init_z);

    q_sol = x_sol(1:OCP.system.nDoF,:);
    q_dot_sol = x_sol(OCP.system.nDoF+1:end,:);

    if OCP.useSplineInputs
        u_sol = (B*u_sol_z.').';
    else
        u_sol = u_sol_z;
    end

    % Plot solution data
    OCP_ODE.plotConstraintResiduals(x_sol, u_sol_z, "figureName", "Constr. Res. Solution");
    plotOCPqu(OCP_ODE, q_sol, u_sol, "q_dot", q_dot_sol, "plotDerivatives", true);

    if OCP.iRC(5)
        fh = plotOCPTCPTraj(MBSim, OCP, q_sol);
    end
end


%% Define DEL OCP Solver
OCP_DEL = OCP;
OCP_DEL.Name ="VI";
OCP_DEL.discretization = OCPIntegratorVI;

OCP_DEL = OCP_DEL.initSolver;

% Solve DEL OCP
% with weights and x_TCP specified in OCP object

% Initial guess objective components
if ~OCP_DEL.useSplineInputs
    disp("Objective function components initial guess:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call( ...
        {quMat2XVec(q_init, u_init), OCP.x_TCP_F, OCP.wRC} ...
        )));
end

% Plot constraint residuals of the initial guess
OCP_DEL.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

[q_sol, u_sol_z, sol, stats] = OCP_DEL.solve(q_init, u_init_z);

% Plot solution data
OCP_DEL.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");

if OCP.useSplineInputs
    u_sol = (B*u_sol_z.').';
else
    u_sol = u_sol_z;
end

plotOCPqu(OCP_DEL, q_sol, u_sol, "plotDerivatives", true);

if ~OCP.useSplineInputs
    disp("Objective function components solution:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call( ...
        {quMat2XVec(q_sol, u_sol), OCP.x_TCP_F, OCP.wRC} ...
        )))
end

if OCP.iRC(5)
    fh = plotOCPTCPTraj(MBSim, OCP, q_sol);
end

%% Post-process etc.

disp('Post processing...')
gTCPDes = SE3Matrix(eye(3), OCP_DEL.x_TCP_F);

[q_dot, ~] = diff2ndOrder(q_sol, OCP_DEL.h);

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.results = getSimResFromStateTrajectory(MBSim.system, OCP_DEL.tout, q_sol, q_dot);

MBSimCasadi.plotAll;

% Draw snapshots
fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");
CoordSysSE3(gTCPDes);
MBSimCasadi.drawSnapshots("figure", fig, "nSnapShots", 20);

% Animate results
fig = init3Dplot('Name', "Animation Solution");
CoordSysSE3(gTCPDes);
MBSimCasadi.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");
xlim([-0.2, 0.8]);
ylim([-0.2, 0.3]);
zlim([0, 1]);


%% End script
disp("Finished.")
