%% Optimal Control / Trajectory Generation for a Simple Planar Robot
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

% Add example system folder if it's not on the path
% addpath("exampleSystems");

%% Script settings

% False = use zero initial guess
COMPUTE_IG = 0;

%% Define System

links = systemDef_planarNLinkPendulum("nLinks", 2, "d", 0);
MBSim = MBSimulation(links, "displayInfo", false);

% Visualize system in reference configuration
MBSim.visualizeSystemRefConf();


%% Quick example forward simulation

MBSimFwd = MBSim;
MBSimFwd.simPars.tEnd = 5;

q0 = deg2rad([30,-30]);
MBSimFwd.simPars.q0 = q0;
MBSimFwd.simPars.qDot0 = zeros(2,1);

% Visualize initial config
MBSimFwd.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% Solver settings
MBSimFwd.solver = MBSimIntegratorVarIntBroyden;
MBSimFwd.solver.h = 2^-7;
MBSimFwd.solver.JacobianIterationThreshold = 5;
MBSimFwd.solver.errorMargin = 1e-11;

% Start integration
MBSimFwd = MBSimFwd.simulateSystem;

% Plotting
MBSimFwd.plotAll;

% Animate results
MBSimFwd.animateSimResults("figureName", "AnimVI");

%% Define OCP

OCP = OCPDefinition;
OCP.MBSys = elara.SystemSym(links);

OCP.q0    = [pi/2, 0];
OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

OCP.u0 = [];

% End time, sample time
OCP.h = 1e-2;
OCP.tF = 1;


% Desired end configuration
OCP.qF = [-pi/2, 0];
OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = false;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 25;

OCP.simPars = MBSim.simPars;

% Running cost
OCP.wRC = [ % Weights
    1/2 % Norm u
    0   % Norm u_dot
    0   % Norm u_ddot
    0   % Norm q_ddot
    0   % TCP error
    ];
OCP.iRC = logical(OCP.wRC); % Defines which cost terms are active

% Final Cost
OCP.wFC = zeros(3,1); % Weights
OCP.iFC = zeros(3,1); % Defines which cost terms are active

OCP.uMin = ones(2,1)*-25;
OCP.uMax = ones(2,1)*+25;

OCP.qMin = [-inf, -4];
OCP.qMax = [inf, 0.1];


%% Visualize reference configuration and target position

figure("Name", "initial/final config");
tiledlayout;
nexttile;
init3Dplot("createFigure", false);
MBSim.visualizeSystemConfig(OCP.q0, "createFigure", false);
title("Initial Configuration");
nexttile;
MBSim.visualizeSystemConfig(OCP.qF, "createFigure", false);
title("Final Configuration");
init3Dplot("createFigure", false);


%% Compute Initial Guess

ANIMATE_IG = 1;

if COMPUTE_IG
    [q_init, u_init, MBSimIG, qOptStatic, uOptStatic] = OCPComputeInitialGuess_InvDyn(MBSim, OCP);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");
    if ~isempty(OCP.x_TCP_F)
        coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
    end
    drawWorkspace(OCP.workSpaceDef, "createFigure", false);

    fh_IG = plotOCPqu(OCP, q_init, u_init, "figureName", "Initial Guess");

    % Animate results
    if ANIMATE_IG
        fig = init3Dplot('Name', "Animation Initial Guess");
        if ~isempty(OCP.x_TCP_F)
            coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
        end
        drawWorkspace(OCP.workSpaceDef, "createFigure", false);
        MBSimIG.animateSimResults("figure", fig);
    end
else
    q_init = repmat(OCP.q0, [1,OCP.nSteps+1]);
    u_init = zeros(MBSim.MBSys.nInputs,OCP.nSteps+1);
end

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
    colororder(lines(MBSim.MBSys.nInputs));
    legend;
    title("Spline Fit");

    nexttile;
    plot(OCP.tout, abs(u_init.'-B*u_init_z.'));
    grid on;
    title("Fit Error");
else
    u_init_z = u_init;
end



%% Define ODE OCP Solver

OCP_ODE = OCP;
OCP_ODE.discretization = OCPIntegratorRK("RK4");
OCP_ODE.Name = "RK4";

qDotInit = diff2ndOrder(q_init, OCP_ODE.h);
x_init = [q_init;qDotInit];

OCP_ODE = OCP_ODE.initSolver;
OCP_ODE.plotConstraintResiduals(x_init, u_init_z, "figureName", "Constr. Res. IG");

% Solve ODE OCP
[x_sol, u_sol_z, sol, stats] = OCP_ODE.solve(x_init, u_init_z);

q_sol = x_sol(1:OCP_ODE.MBSys.nDoF,:);
q_dot_sol = x_sol(OCP_ODE.MBSys.nDoF+1:end,:);


if OCP.useSplineInputs
    u_sol = (B*u_sol_z.').';
else
    u_sol = u_sol_z;
end

% Plot solution data
OCP_ODE.plotConstraintResiduals(x_sol, u_sol_z, "figureName", "Constr. Res. Solution");
plotOCPqu(OCP_ODE, q_sol, u_sol, "q_dot", q_dot_sol, "plotDerivatives", true);

%% Define DEL OCP Solver

OCP_DEL = OCP;
OCP_DEL.Name ="VI";
OCP_DEL.discretization = OCPIntegratorVI;

OCP_DEL = OCP_DEL.initSolver("useCasadiStepFunctions", true);

% Solve DEL OCP

% Initial guess objective components
if ~OCP_DEL.useSplineInputs
    disp("Objective function components initial guess:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call({quMat2XVec(q_init, u_init), OCP.x_TCP_F, OCP.wRC}) ))
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

if ~OCP_DEL.useSplineInputs
    disp("Objective function components solution:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call({quMat2XVec(q_sol, u_sol), OCP.x_TCP_F, OCP.wRC}) ))
end

%% Post-process etc.

disp('Post processing...')
gTCPDes = SE3Matrix(eye(3), OCP_DEL.x_TCP_F);

[q_dot, ~] = diff2ndOrder(q_sol, OCP_DEL.h);

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.simRes = getSimResFromStateTrajectory(MBSim.MBSys, OCP_DEL.tout, q_sol, q_dot);

MBSimCasadi.plotAll;

% Draw snapshots
fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");
coordSysSE3(gTCPDes);
MBSimCasadi.drawSnapshots("figure", fig, "nSnapShots", 15);
TCPTraj = squeeze(MBSimCasadi.simRes.g(1:3,4,end,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');

%% Animate results
fig = init3Dplot('Name', "Animation Solution");
coordSysSE3(gTCPDes);
MBSimCasadi.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");


%% End script
disp("Finished.")
