%% Example Simulation of a PD-Controlled "Surgical" Rigid-Soft Manipulator
% With two rigid base links and a flexible "end effector"
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
addpath("exampleSystems");

%% Define System
links = systemDef_rigid_flexible_robot("dBeam", ones(6,1)*1e-3);

MBSim = MBSimulation(links, "displayInfo", true);


% Visualize reference configuration
MBSim.visualizeSystemRefConf;

% Visualize desired configuration
qDes = deg2rad([45,-45, 90,-80]);
MBSim.visualizeSystemConfig([qDes, zeros(1,2*links(end).nSeg+1)]);


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 5;

% Initial configuration
MBSim.simPars.q0    = zeros(MBSim.MBSys.nDoF,1);
%MBSim.simPars.q0(2) = pi/4;
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(MBSim.simPars.q0, "figureName", "visInitConf");
title("Initial Configuration")


%% Integration with variational integrator

% Add Pseudo-PD Control via joint Stiffness and Dissipation
nLinksRigid = 4;
MBSim.MBSys.dSys(1:nLinksRigid) = ones(nLinksRigid,1) * 30;
MBSim.MBSys.cSys(1:nLinksRigid) = ones(nLinksRigid,1) * 100;
MBSim.MBSys.qRef(1:nLinksRigid) = qDes;

MBSimVI = MBSim;

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-8;
MBSimVI.solver.JacobianIterationThreshold = 5;
MBSimVI.solver.errorMargin = 5e-12;
MBSimVI.solver.aTrapez = 0;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode15s";
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-3;
MBSimODE.solver.odeObject.RelativeTolerance = 1e-3;

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
