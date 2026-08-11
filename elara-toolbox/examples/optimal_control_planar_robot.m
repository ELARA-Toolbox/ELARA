%% Optimal Control / Trajectory Generation for a Simple Planar Robot
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
COMPUTE_IG = 0;

%% Define System

links = systemDef_planarNLinkPendulum("nLinks", 2, "d", 0);

% Visualize system in reference configuration
MBSim = elara.Simulation(links);
MBSim.visualizeSystemRefConf();


%% Quick example forward simulation

MBSimFwd = MBSim;
MBSimFwd.parameters.tEnd = 5;

q0 = deg2rad([30; -30]);
MBSimFwd.parameters.q0 = q0;
MBSimFwd.parameters.qDot0 = zeros(2,1);

% Visualize the initial configuration
MBSimFwd.visualizeSystemConfig(q0, ...
    "figureName", "Visualization Initial Config Fwd Sim");
title("Initial Configuration")

% Solver settings
MBSimFwd.integrator = elara.integration.VIBroyden;
MBSimFwd.integrator.h = 2^-7;
MBSimFwd.integrator.JacobianIterationThreshold = 5;
MBSimFwd.integrator.tolerance = 1e-11;

% Start integration
MBSimFwd = MBSimFwd.simulateSystem;

% Plotting
MBSimFwd.plotAll;

% Animate results
MBSimFwd.animateSimResults("figureName", "Animation Fwd Sim");

%% Define OCP

OCP = elara.ocp.Problem(links);

OCP.q0    = [pi/2; 0];
OCP.qDot0 = zeros(OCP.systemNum.nDoF,1); % Initial velocity
OCP.qDotF = zeros(OCP.systemNum.nDoF,1); % Final velocity

OCP.u0 = [];

% End time, sample time
OCP.h = 1e-2;
OCP.tEnd = 1;

% Desired end configuration
OCP.qF = [-pi/2; 0];
OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = false;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 25;

% Running cost
OCP.runningCostWeights = [ % Weights
    1/2 % Norm u
    0   % Norm u_dot
    0   % Norm u_ddot
    0   % Norm q_ddot
    0   % TCP error
    ];
OCP.runningCostActive = logical(OCP.runningCostWeights); % Defines which cost terms are active

% Final Cost
OCP.finalCostWeights = zeros(3,1); % Weights
OCP.finalCostActive = zeros(3,1); % Defines which cost terms are active

OCP.uMin = ones(2,1)*-25;
OCP.uMax = ones(2,1)*+25;

OCP.qMin = [-inf; -4];
OCP.qMax = [inf; 0.1];


%% Visualize reference configuration and target position

figure("Name", "Initial and Final Configurations");
tiledlayout;
nexttile;
elara.visualization.initializeAxes("createFigure", false);
MBSim.visualizeSystemConfig(OCP.q0, "createFigure", false);
title("Initial Configuration");
nexttile;
MBSim.visualizeSystemConfig(OCP.qF, "createFigure", false);
title("Final Configuration");
elara.visualization.initializeAxes("createFigure", false);


%% Compute Initial Guess

ANIMATE_IG = 1;

if COMPUTE_IG
    [q_init, ~, u_init, MBSimIG, qOptStatic] = ...
        elara.ocp.computeInitialGuessInvDyn(OCP);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");
    if ~isempty(OCP.x_TCP_F)
        elara.visualization.CoordinateFrame( ...
            elara.SE3.matrix(eye(3), OCP.x_TCP_F));
    end
    OCP.workspace.visualize("createFigure", false);

    elara.ocp.plot.coordinatesInputs(OCP, q_init, u_init, "figureName", "Initial Guess");

    % Animate results
    if ANIMATE_IG
        fig = elara.visualization.initializeAxes('Name', "Animation Initial Guess");
        if ~isempty(OCP.x_TCP_F)
            elara.visualization.CoordinateFrame( ...
                elara.SE3.matrix(eye(3), OCP.x_TCP_F));
        end
        OCP.workspace.visualize("createFigure", false);
        MBSimIG.animateSimResults("figure", fig);
    end
else
    q_init = repmat(OCP.q0, [1,OCP.nSteps+1]);
    u_init = zeros(OCP.systemNum.nInputs,OCP.nSteps+1);
end

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



%% Define ODE OCP Solver

OCP_ODE = OCP;
OCP_ODE.discretization = elara.ocp.DiscretizationRK("RK4");
OCP_ODE.Name = "RK4";

q_dot_init = diff2ndOrder(q_init, OCP_ODE.h);
x_init = [q_init; q_dot_init];

OCP_ODE = OCP_ODE.initSolver;
OCP_ODE.plotConstraintResiduals(x_init, u_init_z, "figureName", "Constr. Res. IG");

% Display cost values for the initial guess trajectory
[J_init, cR_init, cF_init] = OCP_ODE.evaluateObjectiveComponents(x_init, u_init_z);
disp("Objective ODE initial guess:")
disp(table(J_init, cR_init, cF_init, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));

% Solve ODE OCP
[x_sol, u_sol, u_sol_z] = OCP_ODE.solve(x_init, u_init_z);

q_sol = x_sol(1:OCP_ODE.systemNum.nDoF,:);
q_dot_sol = x_sol(OCP_ODE.systemNum.nDoF+1:end,:);

% Plot solution data
OCP_ODE.plotConstraintResiduals(x_sol, u_sol_z, "figureName", "Constr. Res. Solution");
elara.ocp.plot.coordinatesInputs(OCP_ODE, q_sol, u_sol, "q_dot", q_dot_sol, "plotDerivatives", true);

% Display cost values for the solution trajectory
[J_sol, cR_sol, cF_sol] = OCP_ODE.evaluateObjectiveComponents(x_sol, u_sol_z);
disp("Objective ODE solution:")
disp(table(J_sol, cR_sol, cF_sol, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));


%% Define DEL OCP Solver

OCP_DEL = OCP;
OCP_DEL.Name ="VI";
OCP_DEL.discretization = elara.ocp.DiscretizationVI;

OCP_DEL = OCP_DEL.initSolver("useCasadiStepFunctions", true);

% Display cost values for the initial guess trajectory
[J_init, cR_init, cF_init] = OCP_DEL.evaluateObjectiveComponents(q_init, u_init_z);
disp("Objective DEL initial guess:")
disp(table(J_init, cR_init, cF_init, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));

% Plot constraint residuals of the initial guess
OCP_DEL.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

% Solve DEL OCP
[q_sol, u_sol, u_sol_z] = OCP_DEL.solve(q_init, u_init_z);

% Plot solution data
OCP_DEL.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");
elara.ocp.plot.coordinatesInputs(OCP_DEL, q_sol, u_sol, "plotDerivatives", true);

% Display cost values for the solution trajectory
[J_sol, cR_sol, cF_sol] = OCP_DEL.evaluateObjectiveComponents(q_sol, u_sol_z);
disp("Objective DEL solution:")
disp(table(J_sol, cR_sol, cF_sol, 'VariableNames', ["Total Cost", "Running Cost", "Final Cost"]));


%% Post-process and visualize the solution

disp('Post processing...')
gTCPDes = elara.SE3.matrix(eye(3), OCP_DEL.x_TCP_F);

MBSimOCP = MBSim;
MBSimOCP.Name = "Optimization";
MBSimOCP.results = elara.SimulationResults.fromStateTrajectory( ...
    OCP_DEL.systemNum, OCP_DEL.tout, q_sol, "finiteDifferenceOrder", 2);
MBSimOCP.plotAll;

% Draw snapshots
fig = elara.visualization.initializeAxes( ...
    'Name', "Snapshots Solution", "NumberTitle", "off");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.drawSnapshots("figure", fig, "nSnapShots", 15);
TCPTraj = squeeze(MBSimOCP.results.g(1:3,4,end,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');

%% Animate results
fig = elara.visualization.initializeAxes('Name', "Animation Solution");
elara.visualization.CoordinateFrame(gTCPDes);
MBSimOCP.animateSimResults("figure", fig, "saveMovie", false, ...
    "fileName", "example_optControl_planarRobot");


%% End script
disp("Finished.")
