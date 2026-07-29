%% Example Simulation of a PD-Controlled "Surgical" Rigid-Soft Manipulator
% With two rigid base links and a flexible "end effector"
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
links = systemDef_rigid_flexible_robot("dBeam", ones(6,1)*1e-3);

MBSim = elara.Simulation(links, "displayInfo", true);


% Visualize reference configuration
MBSim.visualizeSystemRefConf;

% Visualize desired configuration
qDes = deg2rad([45,-45, 90,-80]);
MBSim.visualizeSystemConfig([qDes, zeros(1,2*links(end).nSegments+1)]);


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 5;

% Initial configuration
MBSim.parameters.q0    = zeros(MBSim.system.nDoF,1);
%MBSim.parameters.q0(2) = pi/4;
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(MBSim.parameters.q0, "figureName", "visInitConf");
title("Initial Configuration")


%% Integration with variational integrator

% Add Pseudo-PD Control via joint Stiffness and Dissipation
nLinksRigid = 4;
MBSim.system.dSys(1:nLinksRigid) = ones(nLinksRigid,1) * 30;
MBSim.system.cSys(1:nLinksRigid) = ones(nLinksRigid,1) * 100;
MBSim.system.qRef(1:nLinksRigid) = qDes;

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-8;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 5e-12;
MBSimVI.integrator.useFirstOrderDissipation = true;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "ode15s";
MBSimODE.integrator.odeObject.AbsoluteTolerance = 1e-3;
MBSimODE.integrator.odeObject.RelativeTolerance = 1e-3;

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
