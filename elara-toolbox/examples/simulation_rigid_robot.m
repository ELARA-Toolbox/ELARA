%% Example Simulation of a Rigid Robot
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

links = systemDef_rigid_robot;
MBSim = elara.Simulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 10;

% Initial configuration
q0 = deg2rad([30,-30, -60]);
MBSim.parameters.q0 = q0;
MBSim.parameters.qDot0 = zeros(3,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
MBSim.parameters.uConst = [1,1,1]*0;

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-8;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 1e-11;
MBSimVI.integrator.useFirstOrderDissipation = false;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
elara.plot.energies(MBSimVI.results);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "ode45";
MBSimODE.integrator.odeObject.AbsoluteTolerance = 1e-8;
MBSimODE.integrator.odeObject.RelativeTolerance = 1e-8;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
elara.plot.energies(MBSimODE.results);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");


%% End script
disp("Finished.")
