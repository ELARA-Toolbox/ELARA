%% Optimal Control / Trajectory Generation for a Rigid Two-Link Robot
% In this example, the robot must follow a prescribed TCP trajectory while
% avoiding an obstacle in the workspace. The workspace constraints are 
% enforced at the discretization nodes
% For comparison, the problem is solved with an RK2 and the VI
% discretization.
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
COMPUTE_IG = 1;

%% Define System

links = systemDef_rigid_robot("d", 0);


%% Define OCP

OCP = elara.ocp.Problem(links);

OCP.q0    = zeros(OCP.systemNum.nDoF,1); % Initial configuration
OCP.qDot0 = zeros(OCP.systemNum.nDoF,1); % Initial velocity
OCP.qDotF = zeros(OCP.systemNum.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = ones(OCP.systemNum.nInputs,1)*-100;
OCP.uMax = ones(OCP.systemNum.nInputs,1)*100;

% End time, sample time
OCP.h = 1e-2;
OCP.tEnd = 2;

% Desired TCP position
OCP.x_TCP_F = [0.7; 0.2; 0.3];

% Running cost
OCP.runningCostWeights = [
    1e-2/2 % Norm u
    1e-5   % Norm u_dot
    1e-5   % Norm u_ddot
    1e-5   % Norm q_ddot
    1e4    % TCP error
    ];
OCP.runningCostActive = logical(OCP.runningCostWeights); % Defines which cost terms are active

% No final time cost term
OCP.finalCostActive = zeros(3,1); % Defines which cost terms are active

OCP.addTCPFinalTimeConstraint = false;
OCP.tPreAct  = 5*2^-5;
OCP.tPostAct = 2*2^-5;

OCP.qMin = ones(OCP.systemNum.nDoF, 1)*-2*pi;
OCP.qMax = ones(OCP.systemNum.nDoF, 1)*2*pi;

% Control trajectory parameterization with splines
OCP.useSplineInputs = true;
OCP.nInputSplinePoints = 30;
OCP.inputSplineOrder = 3;

% NLP object / solver options
OCP.nlpOptions.expand = false;


%% Generate desired TCP trajectory
% As a linear point-to–point trajectory
OCP.x_TCP_traj = elara.ocp.computeLinearReferenceTCPTrajectory(OCP);


%% Define Workspace

% Add an obstacle in the workspace represented by a simple box
OCP.workspace = elara.Workspace;
OCP.workspace = OCP.workspace.addBoxSideLengths( ...
    [0.35, 0.1, 1.0], ...  % Box center position
    zeros(3,1), ...       % Rotation (Euler angles)
    [0.25, 0.3, 0.3], ... % Side lengths
    0 ...                 % Object type: obstacle (type 0)
    );


%% Visualize reference configuration and target position

% Get Simulation object from OCP
MBSim = OCP.getSimulationObject;

elara.visualization.initializeAxes( ...
    "Name", "System Visualization", "NumberTitle", "off");

% Visualize system in reference configuration
MBSim.visualizeSystemRefConf("createFigure", false);

% Mark final TCP position
elara.visualization.CoordinateFrame(elara.SE3.matrix(eye(3), OCP.x_TCP_F));

% Visualize workspace
OCP.workspace.visualize("createFigure", false);

% Draw desired trajectory
plot3(OCP.x_TCP_traj(1,:), OCP.x_TCP_traj(2,:), OCP.x_TCP_traj(3,:), "-o");

legend("Obstacle", "Desired TCP Trajectory");
xlim([-0.2, 0.8]);
ylim([-0.3, 0.5]);
zlim([0, 1.3]);


%% Initial Guess based on ODE Inverse Dynamics

% Compute Initial Guess
if COMPUTE_IG
    [q_init, q_dot_init, u_init, MBSimIG] = elara.ocp.computeInitialGuessInvDyn( ...
        OCP, "createDebugPlots", false, "invDynMethod", "ODE");

    elara.ocp.plot.coordinatesInputs(OCP, q_init, u_init, ...
        "figureName", "Initial Guess", "plotDerivatives", true);

    % Animate results
    fig = elara.visualization.initializeAxes('Name', "Animation Initial Guess");
    elara.visualization.CoordinateFrame(elara.SE3.matrix(eye(3), OCP.x_TCP_F));
    OCP.workspace.visualize("createFigure", false);
    MBSimIG.animateSimResults("figure", fig);
else
    q_init = repmat(OCP.q0, [1,OCP.nSteps+1]);
    q_dot_init = zeros(OCP.systemNum.nDoF, OCP.nSteps+1);
    u_init = zeros(OCP.systemNum.nInputs, OCP.nSteps+1);
end

if OCP.useSplineInputs
    % Compute control points for initial guess
    B = OCP.getInputSplineBasisMatrix;
    u_init_z =  (B \ u_init.').';

    % Plot initial guess fit
    figure("Name", "Initial Guess B-spline Fit");
    plot(OCP.tout, u_init, "-.x", "DisplayName", "Original Data");
    hold on;
    plot(OCP.tout, B*u_init_z.', "--o", "DisplayName", "Fitted Spline");

    grid on;
    colororder(lines(OCP.systemNum.nInputs));
    legend;
else
    u_init_z = u_init;
end


%% Define ODE OCP Solver

OCP_ODE = OCP;
OCP_ODE.discretization = elara.ocp.DiscretizationRK("RK2");
OCP_ODE.Name = "RK2";
x_init = [q_init; q_dot_init];

% Initialize the ODE solver.
% Additionally show the constraint Jacobian to inspect its structure and
% make sure it has approximately block-diagonal structure
OCP_ODE = OCP_ODE.initSolver("showDebugPlots", true);

% Plot constraint residuals of the initial guess: this will reveal a
% violation of the workspace constraints
OCP_ODE.plotConstraintResiduals(x_init, u_init_z, "figureName", "Constr. Res. IG");

% Display cost values for the initial guess trajectory
[J_init, cR_init, cF_init] = OCP_ODE.evaluateObjectiveComponents(x_init, u_init_z);
disp("Objective ODE initial guess:")
disp(table(J_init, cR_init, cF_init, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));

% Solve ODE OCP
[x_sol, u_sol, u_sol_z] = OCP_ODE.solve(x_init, u_init_z);

q_sol = x_sol(1:OCP.systemNum.nDoF,:);
q_dot_sol = x_sol(OCP.systemNum.nDoF+1:end,:);

% Plot solution data
OCP_ODE.plotConstraintResiduals(x_sol, u_sol_z, "figureName", "Constr. Res. Solution");
elara.ocp.plot.coordinatesInputs(OCP_ODE, q_sol, u_sol, "q_dot", q_dot_sol, "plotDerivatives", true);

if OCP.runningCostActive(5)
    fh = elara.ocp.plot.TCPTrajectory(OCP, q_sol);
end

% Display cost values for the solution trajectory
[J_sol, cR_sol, cF_sol] = OCP_ODE.evaluateObjectiveComponents(x_sol, u_sol_z);
disp("Objective ODE solution:")
disp(table(J_sol, cR_sol, cF_sol, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));


%% Define DEL OCP Solver
OCP_DEL = OCP;
OCP_DEL.Name ="VI";
OCP_DEL.discretization = elara.ocp.DiscretizationVI;

% Initialize DEL solver; again inspect the constraint Jacobian
OCP_DEL = OCP_DEL.initSolver("showDebugPlots", true);

% Solve DEL OCP
% with weights and x_TCP specified in OCP object

% Display cost values for the initial guess trajectory
[J_init, cR_init, cF_init] = OCP_DEL.evaluateObjectiveComponents(q_init, u_init_z);
disp("Objective DEL initial guess:")
disp(table(J_init, cR_init, cF_init, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));

% Plot constraint residuals of the initial guess
OCP_DEL.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

[q_sol, u_sol, u_sol_z] = OCP_DEL.solve(q_init, u_init_z);

% Plot solution data
OCP_DEL.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");
elara.ocp.plot.coordinatesInputs(OCP_DEL, q_sol, u_sol, "plotDerivatives", true);

if OCP.runningCostActive(5)
    fh = elara.ocp.plot.TCPTrajectory(OCP, q_sol);
end

% Display cost values for the solution trajectory
[J_sol, cR_sol, cF_sol] = OCP_DEL.evaluateObjectiveComponents(q_sol, u_sol_z);
disp("Objective DEL solution:")
disp(table(J_sol, cR_sol, cF_sol, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));


%% Post-process and visualize the solution

disp('Post processing...')

MBSimOCP = OCP.getSimulationObject();
MBSimOCP.Name = "Optimization";
MBSimOCP.results = elara.SimulationResults.fromStateTrajectory( ...
    OCP_DEL.systemNum, OCP_DEL.tout, q_sol, "finiteDifferenceOrder", 2);

MBSimOCP.plotAll;

% Draw snapshots
gTCPDes = elara.SE3.matrix(eye(3), OCP_DEL.x_TCP_F);
fig = elara.visualization.initializeAxes( ...
    'Name', "Snapshots Solution", "NumberTitle", "off");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.drawSnapshots("figure", fig, "nSnapShots", 20);

% Animate results
fig = elara.visualization.initializeAxes('Name', "Animation Solution");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.animateSimResults("figure", fig, "saveMovie", false, ...
    "fileName", "example_optControl_rigidRobot");
xlim([-0.2, 0.8]);
ylim([-0.2, 0.3]);
zlim([0, 1]);


%% End script
disp("Finished.")
