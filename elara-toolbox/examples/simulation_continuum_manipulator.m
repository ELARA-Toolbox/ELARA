%% Example Simulation of a one-link tendon-actuated continuum manipulator
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
addpath("exampleSystems");

%% Define System

links = systemDef_continuum_manipulator;
MBSim = MBSimulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;

%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 5;

% Initial configuration
MBSim.simPars.q0    = zeros(MBSim.MBSys.nDoF,1);
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);
%MBSim.simPars.q0(1) = 1;

% Visualize initial config
MBSim.visualizeSystemConfig(MBSim.simPars.q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
uConst = [0,25,0,0].';
MBSim.simPars.uSampleTimes  = linspace(0, MBSim.simPars.tEnd, 100);
MBSim.simPars.uSampleValues = repmat(((1-cos(pi*MBSim.simPars.uSampleTimes ./ MBSim.simPars.tEnd ))/2).', [4,1]) .* uConst;
%MBSim.simPars.uConst = uConst;

figure("Name", "System Inputs");
plot(MBSim.simPars.uSampleTimes, MBSim.simPars.uSampleValues);
grid on;
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("Inputs $u$", "Interpreter", "latex");
legend(arrayfun(@(x) sprintf("Input $u_%d$", x), 1:MBSim.MBSys.nInputs), "Interpreter", "latex");

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-9;
MBSimVI.solver.JacobianIterationThreshold = 3;
MBSimVI.solver.errorMargin = 1e-9;
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
%MBSimODE.simPars.tEnd = 10;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode15s";
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-5;
MBSimODE.solver.odeObject.RelativeTolerance = 1e-5;

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
