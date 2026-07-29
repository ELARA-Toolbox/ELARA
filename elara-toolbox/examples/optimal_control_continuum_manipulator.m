%% Optimal Control / Trajectory Generation for a Continuum Manipulator
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

links = systemDef_continuum_manipulator("nSegments", 4);

MBSim = elara.Simulation(links, "displayInfo", false);


%% Define OCP

OCP = elara.ocp.Problem;
OCP.system = elara.SystemSym(links);

OCP.q0    = zeros(MBSim.system.nDoF,1);
OCP.qDot0 = zeros(MBSim.system.nDoF,1); % Initial velocity
OCP.qDotF = zeros(MBSim.system.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = ones(MBSim.system.nInputs,1)*-1e-3; % Add small slack for improved convergence
OCP.uMax = ones(MBSim.system.nInputs,1)*100;

% End time, sample time
OCP.h = 2^-6;
OCP.tEnd = 2;

% Desired TCP pose
OCP.x_TCP_F = [0.55; 0.3; 0.05];
OCP.R_TCP_F = []; % Rotation arbitrary

OCP.simPars = MBSim.parameters;

% Running cost
OCP.runningCostWeights = [ % Weights
    1e-3  % Norm u
    1e-2  % Norm u_dot
    5e-2  % Norm u_ddot
    1e-0  % Norm q_ddot
    0     % TCP error
    ];
OCP.runningCostActive = logical(OCP.runningCostWeights);  % Defines which cost terms are active

% Final time cost term
OCP.finalCostWeights = [ % Weights
    0     % Norm u
    0     % Norm q
    1e5   % TCP Error
    ];
OCP.finalCostActive = logical(OCP.finalCostActive);  % Defines which cost terms are active

OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = true;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 30;

% NLP object / solver options
OCP.nlpOptions.expand = false;

%% Visualize reference configuration and target position

[~, vis] = MBSim.visualizeSystemRefConf();
CoordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
OCP.workspace.visualize("createFigure", false);

%% Compute Initial Guess

OCP.tPreAct  = 0;
OCP.tPostAct = 0;

if COMPUTE_IG
    [q_init, qd_init, u_init, MBSimIG, qOptStatic, uOptStatic] = elara.ocp.computeInitialGuessInvDyn( ...
        MBSim, OCP, "invDynMethod", "ODE", "createDebugPlots", true);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");

    % Animate results
    fig = init3Dplot('Name', "Animation Initial Guess");
    CoordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
    OCP.workspace.visualize("createFigure", false);
    MBSimIG.animateSimResults("figure", fig);
else
    q_init = repmat(OCP.q0, [1, OCP.nSteps+1]);
    u_init = repmat(OCP.u0, [1, OCP.nSteps+1]);
end

plotOCPqu(OCP, q_init, u_init, "figureName", "Initial Guess", "plotDerivatives", true);

gOptStatic = MBSim.system.computeFwdKin(qOptStatic);
g_TCP = gOptStatic(:,:,MBSim.system.indexTCPFrame)*MBSim.system.g_B_TCP;
x_TCP_des = g_TCP(1:3, 4);

if OCP.useSplineInputs
    % Compute control points for initial guess
    B = OCP.getInputSplineBasisMatrix;
    u_init_z =  (B \ u_init.').';

    % Plot initial guess fit
    figure("Name", "Initial Guess B-Spline Fit");
    tiledlayout("vertical");
    nexttile;
    plot(OCP.tout, u_init, "-.x", "DisplayName", "Original Data");
    hold on;
    plot(OCP.tout, B*u_init_z.', "--o", "DisplayName", "Fitted Spline");
    grid on;
    colororder(lines(MBSim.system.nInputs));
    legend;
    title("Spline Fit");

    nexttile;
    plot(OCP.tout, abs(u_init.'-B*u_init_z.'));
    grid on;
    title("Fit Error");
else
    u_init_z = u_init;
end


%% Define DEL OCP Solver

OCP.Name = "VI";
OCP.discretization = elara.ocp.IntegratorVI;

OCP = OCP.initSolver("useCasadiStepFunctions", true);

% Plot constraint residuals of the initial guess
OCP.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

% Initial guess objective components
if ~OCP.useSplineInputs
    disp("Objective function components initial guess:")
    disp(cellfun( @(x) full(x), OCP.constrDef.Fun_fComp.call({quMat2XVec(q_init, u_init), OCP.x_TCP_F, OCP.w}) ))
end

% Solve OCP
% with weights and x_TCP specified in OCP object
[q_sol, u_sol_z, sol] = OCP.solve(q_init, u_init_z);

% Plot solution data
OCP.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");

if OCP.useSplineInputs
    u_sol = (B*u_sol_z.').';
else
    u_sol = u_sol_z;
end

plotOCPqu(OCP, q_sol, u_sol, "plotDerivatives", true, "FDOrder", 2);

if ~OCP.useSplineInputs
    disp("Objective function components solution:")
    disp(cellfun( @(x) full(x), OCP.constrDef.Fun_fComp.call({quMat2XVec(q_sol, u_sol), OCP.x_TCP_F, OCP.w}) ))
end


%% Post-process etc.

disp('Post processing...')
gTCPDes = SE3Matrix(eye(3), OCP.x_TCP_F);

q_dot_Sol = diff(q_sol, 1, 2) / OCP.h;
q_dot_Sol_full = [q_dot_Sol, nan(OCP.system.nDoF,1)];

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.results = getSimResFromStateTrajectory(MBSim.system, OCP.tout, q_sol, q_dot_Sol_full);
MBSimCasadi.plotAll;

% Draw snapshots
fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");
CoordSysSE3(gTCPDes);
MBSimCasadi.drawSnapshots("figure", fig, "nSnapShots", 20);
TCPTraj = squeeze(MBSimCasadi.results.g(1:3,4,end,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');

% Animate results
fig = init3Dplot('Name', "Animation Solution");
CoordSysSE3(gTCPDes);
MBSimCasadi.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");

%% End script
disp("Finished.")
