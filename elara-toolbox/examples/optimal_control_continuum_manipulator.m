%% Optimal Control / Trajectory Generation for a Continuum Manipulator
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "example-systems"));

%% Script settings

% False = use zero initial guess
COMPUTE_IG = true;

%% Define System

link = systemDef_continuum_manipulator("nSegments", 5);


%% Define OCP

OCP = elara.ocp.Problem(link);

OCP.q0    = zeros(OCP.systemNum.nDoF,1);
OCP.qDot0 = zeros(OCP.systemNum.nDoF,1); % Initial velocity
OCP.qDotF = zeros(OCP.systemNum.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = ones(OCP.systemNum.nInputs,1)*-1e-3; % Add small slack for improved convergence
OCP.uMax = ones(OCP.systemNum.nInputs,1)*100;

% End time, sample time
OCP.h = 2^-6;
OCP.tEnd = 2;

% Desired TCP position
OCP.x_TCP_F = [-0.3; -0.1; 0.35];

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
    1e8   % TCP Error
    ];
OCP.finalCostActive = logical(OCP.finalCostWeights);  % Defines which cost terms are active

OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = true;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 30;

% NLP object/solver options
% Expand the CasADi MX graph; this results in longer NLP construction time,
% but significantly faster solution
OCP.nlpOptions.expand = true;

% Use a fixed common objective scale instead of guess-dependent scaling.
% This improves conditioning without changing the cost ratios or minimizer.
OCP.nlpOptions.ipopt.nlp_scaling_method = 'none';
OCP.nlpOptions.ipopt.obj_scaling_factor = 1e-2;

%% Visualize reference configuration and target position

% Get Simulation object from OCP
MBSim = OCP.getSimulationObject;

MBSim.visualizeSystemRefConf();
elara.visualization.CoordinateFrame(elara.SE3.matrix(eye(3), OCP.x_TCP_F));
OCP.workspace.visualize("createFigure", false);
xlim([min(0,OCP.x_TCP_F(1))-0.1, max(0,OCP.x_TCP_F(1))+0.1]);
ylim([min(0,OCP.x_TCP_F(2))-0.1, max(0,OCP.x_TCP_F(2))+0.1]);
zlim([0,0.8]);

%% Compute Initial Guess

OCP.tPreAct  = 0;
OCP.tPostAct = 0;

if COMPUTE_IG
    % A smaller deformation penalty and a scaled static objective help the
    % initial guess find a helical equilibrium instead of the planar branch.
    % Static weights: [control effort; deformation; TCP position error].
    [q_init, ~, u_init, MBSimIG, qOptStatic] = elara.ocp.computeInitialGuessInvDyn( ...
        OCP, "invDynMethod", "ODE", "createDebugPlots", false, ...
        "staticCostWeights", [1e-6; 1e-3; 1e3]);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");

    % Animate results
    fig = elara.visualization.initializeAxes("Name", "Animation Initial Guess");
    elara.visualization.CoordinateFrame(elara.SE3.matrix(eye(3), OCP.x_TCP_F));
    OCP.workspace.visualize("createFigure", false);
    MBSimIG.animateSimResults("figure", fig);
else
    q_init = repmat(OCP.q0, [1, OCP.nSteps+1]);
    u_init = zeros(OCP.systemNum.nInputs, OCP.nSteps+1);
end

elara.ocp.plot.coordinatesInputs(OCP, q_init, u_init, "figureName", "Initial Guess", "plotDerivatives", true);

if OCP.useSplineInputs
    % Compute control points for initial guess
    B = OCP.getInputSplineBasisMatrix;
    u_init_z =  (B \ u_init.').';

    % Plot initial guess fit
    figure("Name", "Initial Guess B-spline Fit");
    tiledlayout("vertical");
    nexttile;
    plot(OCP.tout, u_init, "-.x", "DisplayName", "Original Data");
    hold on;
    plot(OCP.tout, B*u_init_z.', "--o", "DisplayName", "Fitted Spline");
    grid on;
    colororder(lines(OCP.systemNum.nInputs));
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
OCP.discretization = elara.ocp.DiscretizationVI;

OCP = OCP.initSolver("useCasadiStepFunctions", true);

% Plot constraint residuals of the initial guess
OCP.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

% Display cost values for the initial guess trajectory
[J_init, cR_init, cF_init] = OCP.evaluateObjectiveComponents(q_init, u_init_z);
disp("Objective DEL initial guess:")
disp(table(J_init, cR_init, cF_init, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));

% Solve OCP
% with weights and x_TCP specified in OCP object
[q_sol, u_sol, u_sol_z] = OCP.solve(q_init, u_init_z);

% Plot solution data
OCP.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");
elara.ocp.plot.coordinatesInputs(OCP, q_sol, u_sol, "plotDerivatives", true, "FDOrder", 2);

% Display cost values for the solution trajectory
[J_sol, cR_sol, cF_sol] = OCP.evaluateObjectiveComponents(q_sol, u_sol_z);
disp("Objective DEL solution:")
disp(table(J_sol, cR_sol, cF_sol, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));



%% Post-process and visualize the solution

disp('Post processing...')
gTCPDes = elara.SE3.matrix(eye(3), OCP.x_TCP_F);

q_dot_sol = diff(q_sol, 1, 2) / OCP.h;
q_dot_sol_full = [q_dot_sol, nan(OCP.systemNum.nDoF,1)];

MBSimOCP = OCP.getSimulationObject;
MBSimOCP.Name = "Optimization";
MBSimOCP.results = elara.SimulationResults.fromStateTrajectory( ...
    OCP.systemNum, OCP.tout, q_sol, q_dot_sol_full);
MBSimOCP.plotAll;

% Draw snapshots
fig = elara.visualization.initializeAxes( ...
    'Name', "Snapshots Solution", "NumberTitle", "off");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.drawSnapshots("figure", fig, "nSnapShots", 20);
% Include the TCP's offset from the last backbone frame.
gTCP = pagemtimes(MBSimOCP.results.g(:,:,OCP.systemNum.indexTCPFrame,:), ...
    OCP.systemNum.g_B_TCP);
TCPTraj = squeeze(gTCP(1:3,4,1,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');
fprintf("Final TCP position error: %.3f mm\n", 1e3*norm(TCPTraj(:,end)-OCP.x_TCP_F));

% Animate results
fig = elara.visualization.initializeAxes('Name', "Animation Solution");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.animateSimResults("figure", fig, "saveMovie", false, ...
    "fileName", "example_optControl_contManip");

%% End script
disp("Finished.")
