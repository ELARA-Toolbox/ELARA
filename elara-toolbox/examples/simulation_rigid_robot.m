%% Example Simulation of a Rigid Robot
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
addpath("exampleSystems");

%% Define System

links = systemDef_rigid_robot;
MBSim = MBSimulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 10;

% Initial configuration
q0 = deg2rad([30,-30, -60]);
MBSim.simPars.q0 = q0;
MBSim.simPars.qDot0 = zeros(3,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
MBSim.simPars.uConst = [1,1,1]*0;

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-8;
MBSimVI.solver.JacobianIterationThreshold = 5;
MBSimVI.solver.errorMargin = 1e-11;
MBSimVI.solver.aTrapez = 1/2;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
plotEnergies(MBSimVI.simRes);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode45";
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-8;
MBSimODE.solver.odeObject.RelativeTolerance = 1e-8;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
plotEnergies(MBSimODE.simRes);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");


%% End script
disp("Finished.")
