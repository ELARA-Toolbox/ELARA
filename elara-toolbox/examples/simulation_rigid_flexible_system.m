%% Example Simulation of a Rigid-Flexible System
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
addpath("exampleSystems");


%% Define System

links = systemDef_cantilever_system;

MBSim = MBSimulation(links, "displayInfo", true);

RRef0 = [
    0  0 1
    0  1 0
    -1 0 0
    ];
MBSim.MBSys.g0 = SE3Matrix(RRef0, zeros(3,1));

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 10;

% Initial configuration
q0 = MBSim.MBSys.setJointAngles(deg2rad([-30,90]));
MBSim.simPars.q0 = q0;
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

% Visualize initial config
fig = MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
%title("Initial Configuration")
xlim([0, 1.5]);
ylim([-0.2, 0.2]);
zlim([-0.4, 0.4]);
xlabel('$x$ in m','Interpreter','latex');
ylabel('$y$ in m','Interpreter','latex');
zlabel('$z$ in m','Interpreter','latex');
fig.WindowStyle = "normal";
fig.WindowState = "maximized";


%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-10;
MBSimVI.solver.JacobianIterationThreshold = 5;
MBSimVI.solver.errorMargin = 1e-11;
MBSimVI.solver.aTrapez = 0;


% Start integration
MBSimVI = MBSimVI.simulateSystem;

%% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
plotEnergies(MBSimVI.simRes);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;
%MBSimODE.simPars.tEnd = 3;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode15s";
MBSimODE.solver.odeObject.AbsoluteTolerance = 2e-3;
MBSimODE.solver.odeObject.RelativeTolerance = 2e-3;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
plotEnergies(MBSimODE.simRes);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE", ...
    "saveMovie", false, "fileName", "example_rigidFlexible");


%% End script
disp("Finished.")
