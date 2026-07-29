%% Example Simulation of a one-link tendon-actuated continuum manipulator
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

links = systemDef_continuum_manipulator;
MBSim = elara.Simulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;

%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 5;

% Initial configuration
MBSim.parameters.q0    = zeros(MBSim.system.nDoF,1);
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);
%MBSim.parameters.q0(1) = 1;

% Visualize initial config
MBSim.visualizeSystemConfig(MBSim.parameters.q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
uConst = [0,25,0,0].';
MBSim.parameters.uSampleTimes  = linspace(0, MBSim.parameters.tEnd, 100);
MBSim.parameters.uSampleValues = repmat(((1-cos(pi*MBSim.parameters.uSampleTimes ./ MBSim.parameters.tEnd ))/2).', [4,1]) .* uConst;
%MBSim.parameters.uConst = uConst;

figure("Name", "System Inputs");
plot(MBSim.parameters.uSampleTimes, MBSim.parameters.uSampleValues);
grid on;
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("Inputs $u$", "Interpreter", "latex");
legend(arrayfun(@(x) sprintf("Input $u_%d$", x), 1:MBSim.system.nInputs), "Interpreter", "latex");

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-9;
MBSimVI.integrator.JacobianIterationThreshold = 3;
MBSimVI.integrator.tolerance = 1e-9;
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
%MBSimODE.parameters.tEnd = 10;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "ode15s";
MBSimODE.integrator.odeObject.AbsoluteTolerance = 1e-5;
MBSimODE.integrator.odeObject.RelativeTolerance = 1e-5;

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
