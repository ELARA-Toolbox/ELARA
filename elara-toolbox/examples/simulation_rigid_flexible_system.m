%% Example Simulation of a Rigid-Flexible System
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "exampleSystems"));

%% Define System

links = systemDef_cantilever_system;

MBSim = elara.Simulation(links, "displayInfo", true);

RRef0 = [
    0  0 1
    0  1 0
    -1 0 0
    ];
MBSim.system.g0 = elara.SE3.matrix(RRef0, zeros(3,1));

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 10;

% Initial configuration
q0 = MBSim.system.setJointAngles(deg2rad([-30,90]));
MBSim.parameters.q0 = q0;
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);

% Visualize initial config
fig = MBSim.visualizeSystemConfig(q0, "figureName", "Visualization Initial Config");
title("Initial Configuration", "Interpreter", "latex");
xlim([0, 1.5]);
ylim([-0.2, 0.2]);
zlim([-0.4, 0.4]);
xlabel('$x$ in m','Interpreter','latex');
ylabel('$y$ in m','Interpreter','latex');
zlabel('$z$ in m','Interpreter','latex');


%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-10;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 1e-11;
MBSimVI.integrator.useFirstOrderDissipation = true;


% Start integration
MBSimVI = MBSimVI.simulateSystem;

%% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
elara.plot.energies(MBSimVI.results);

% Animate results
MBSimVI.animateSimResults("figureName", "Animation VI");


%% Integration with ODE solver

MBSimODE = MBSim;
%MBSimODE.parameters.tEnd = 3;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "ode15s";
MBSimODE.integrator.odeObject.AbsoluteTolerance = 2e-3;
MBSimODE.integrator.odeObject.RelativeTolerance = 2e-3;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
elara.plot.energies(MBSimODE.results);

% Animate results
MBSimODE.animateSimResults("figureName", "Animation ODE", ...
    "saveMovie", false, "fileName", "example_rigidFlexible");


%% End script
disp("Finished.")
