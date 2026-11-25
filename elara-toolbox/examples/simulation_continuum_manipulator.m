%% Example Simulation of a one-link tendon-actuated continuum manipulator
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

% Make sure example system folder is on the path
addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "example-systems"));

%% Define System

links = systemDef_continuum_manipulator("nSegments",6);
MBSim = elara.Simulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;

%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 5;

% Initial configuration
MBSim.parameters.q0    = zeros(MBSim.system.nDoF,1);
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);

% Visualize the initial configuration
MBSim.visualizeSystemConfig(MBSim.parameters.q0, ...
    "figureName", "Visualization Initial Config");
title("Initial Configuration")

% System inputs
tAct = 2.5;
uConst = [0,15,5,50].';
MBSim.parameters.uSampleTimes  = linspace(0, tAct, 100);
MBSim.parameters.uSampleValues = repmat(((1-cos(pi*MBSim.parameters.uSampleTimes ./ tAct ))/2).', [4,1]) .* uConst;

MBSim.parameters.uSampleTimes  = [MBSim.parameters.uSampleTimes; MBSim.parameters.tEnd];
MBSim.parameters.uSampleValues = [MBSim.parameters.uSampleValues, uConst];

figure("Name", "System Inputs");
plot(MBSim.parameters.uSampleTimes, MBSim.parameters.uSampleValues);
grid on;
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("Inputs $u$", "Interpreter", "latex");
legend(arrayfun(@(x) sprintf("Input $u_%d$", x), 1:MBSim.system.nInputs), "Interpreter", "latex");

% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-12;
MBSimVI.integrator.JacobianIterationThreshold = 3;
MBSimVI.integrator.tolerance = 1e-9;
MBSimVI.integrator.useFirstOrderDissipation = true;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
elara.plot.energies(MBSimVI.results);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");

return;
%% Integration with ODE solver

MBSimODE = MBSim;

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
