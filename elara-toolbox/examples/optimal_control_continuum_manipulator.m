%% Optimal Control / Trajectory Generation for a Continuum Manipulator
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
addpath("exampleSystems");


%% Script settings

% False = use zero initial guess
COMPUTE_IG = 1;

%% Define System

links = systemDef_continuum_manipulator("nSeg", 4);

MBSim = MBSimulation(links, "displayInfo", false);


%% Define OCP

OCP = OCPDefinition;
OCP.MBSys = MBSystemSym(links);

OCP.q0    = zeros(MBSim.MBSys.nDoF,1);
OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = ones(MBSim.MBSys.nInputs,1)*-1e-3; % Add small slack for improved convergence
OCP.uMax = ones(MBSim.MBSys.nInputs,1)*100;

% End time, sample time
OCP.h = 2^-6;
OCP.tF = 2;

% Desired TCP pose
OCP.x_TCP_F = [0.55; 0.3; 0.05];
OCP.R_TCP_F = []; % Rotation arbitrary

OCP.simPars = MBSim.simPars;

OCP.wRC = [
    1e-3  % Norm u
    1e-3  % Norm u_dot
    1e-3  % Norm u_ddot
    1e-2  % Norm q_ddot
    0    % Final TCP error
    ];
OCP.iRC = logical(OCP.wRC);

% Final time cost term
OCP.wFC = [0,0, 1e-3 * 1e8];
OCP.iFC = logical(OCP.iFC);

OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = true;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 30;%round(OCP_DEL.nSteps / 8);

% NLP object / solver options
OCP.nlpOpts.ipopt.warm_start_init_point = 'no';
OCP.nlpOpts.expand = false;
OCP.nlpOpts.ipopt.linear_solver = 'ma97'; % good for stiff systems

%% Visualize reference configuration and target position

[~, vis] = MBSim.visualizeSystemRefConf();
coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
drawWorkspace(OCP.workSpaceDef, "createFigure", false);

%% Compute Initial Guess

OCP.tPreAct  = 0;
OCP.tPostAct = 0;

if COMPUTE_IG
    [q_init, qd_init, u_init, MBSimIG, qOptStatic, uOptStatic] = OCPComputeInitialGuess_InvDyn( ...
        MBSim, OCP, "invDynMethod", "ODE", "createDebugPlots", true);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");

    % Animate results
    fig = init3Dplot('Name', "Animation Initial Guess");%, "WindowStyle","normal");
    coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
    drawWorkspace(OCP.workSpaceDef, "createFigure", false);
    MBSimIG.animateSimResults("figure", fig);
else
    q_init = repmat(OCP.q0, [1, OCP.nSteps+1]);
    u_init = repmat(OCP.u0, [1, OCP.nSteps+1]);
end

plotOCPqu(OCP, q_init, u_init, "figureName", "Initial Guess", "plotDerivatives", true);

gOptStatic = MBSim.MBSys.computeFwdKin(qOptStatic);
g_TCP = gOptStatic(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
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


%% Define DEL OCP Solver

OCP_DEL = OCP;
OCP_DEL.Name = "VI";
OCP_DEL.discretization = OCPIntegratorVI;

OCP_DEL = OCP_DEL.initSolver("useCasadiStepFunctions", true);

% Plot constraint residuals of the initial guess
OCP_DEL.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

% Initial guess objective components
if ~OCP_DEL.useSplineInputs
    disp("Objective function components initial guess:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fComp.call({quMat2XVec(q_init, u_init), OCP.x_TCP_F, OCP_DEL.w}) ))
end

% Solve OCP
% with weights and x_TCP specified in OCP object
[q_sol, u_sol_z, sol] = OCP_DEL.solve(q_init, u_init_z);

% Plot solution data
OCP_DEL.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");

if OCP_DEL.useSplineInputs
    u_sol = (B*u_sol_z.').';
else
    u_sol = u_sol_z;
end

plotOCPqu(OCP_DEL, q_sol, u_sol, "plotDerivatives", true, "FDOrder", 2);

if ~OCP_DEL.useSplineInputs
    disp("Objective function components solution:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fComp.call({quMat2XVec(q_sol, u_sol), OCP.x_TCP_F, OCP.w}) ))
end


%% Post-process etc.

disp('Post processing...')
gTCPDes = SE3Matrix(eye(3), OCP_DEL.x_TCP_F);

q_dot_Sol = diff(q_sol, 1, 2) / OCP_DEL.h;
q_dot_Sol_full = [q_dot_Sol, nan(OCP.MBSys.nDoF,1)];

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.simRes = getSimResFromStateTrajectory(MBSim.MBSys, OCP_DEL.tout, q_sol, q_dot_Sol_full);

MBSimCasadi.plotAll;

% Draw snapshots
fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");%, "WindowStyle","normal");
coordSysSE3(gTCPDes);
drawWorkspace(OCP_DEL.workSpaceDef, "createFigure", false);
if OCP_DEL.nSteps < 50
    nSnapShots = OCP_DEL.nSteps/2+1;
else
    nSnapShots = 20;
end
MBSimCasadi.drawSnapshots("figure", fig, "nSnapShots",nSnapShots);
TCPTraj = squeeze(MBSimCasadi.simRes.g(1:3,4,end,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');

% Animate results
fig = init3Dplot('Name', "Animation Solution");%, "WindowStyle","normal");
coordSysSE3(gTCPDes);
drawWorkspace(OCP.workSpaceDef, "createFigure", false);
MBSimCasadi.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");

%% End script
disp("Finished.")
