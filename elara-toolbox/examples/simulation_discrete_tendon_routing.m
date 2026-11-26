%% Example Simulation of a Discrete Tendon-Routed Continuum Manipulator
% Requires Symbolic Math Toolbox for the initial continuum-manipulator
% system definition, before its tendon routing is replaced by discrete disks.
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

link = systemDef_continuum_manipulator;

d_tendon = 0.02;

nDisks = 10;
nTendons = 3;

gBackboneTendon = repmat(eye(4), 1,1, nTendons, nDisks);
gBackboneTendon(1:3,4,1,:) = repmat(d_tendon * [1; 0; 0], 1,1, nDisks);
gBackboneTendon(1:3,4,2,:) = repmat(d_tendon * [cos(pi/3); sin(pi/3); 0],1,1,nDisks);
gBackboneTendon(1:3,4,3,:) = repmat(d_tendon * [cos(pi/3); -sin(pi/3); 0],1,1,nDisks);

terminationDisks = nDisks * ones(nTendons,1);
sDisks = linspace(0, link.L,nDisks);

link.tendonActuation = elara.DiscreteTendonActuation( terminationDisks, sDisks, gBackboneTendon);

MBSim = elara.Simulation(link, "displayInfo", true);

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
% The discrete model applies tendon loads at disk locations, so use a
% lower input level than the continuous-routing example.
uConst = [0,1.5,2].';
MBSim.parameters.uSampleTimes = linspace(0, MBSim.parameters.tEnd, 100);
MBSim.parameters.uSampleValues = repmat( ...
    ((1-cos(pi*MBSim.parameters.uSampleTimes ./ MBSim.parameters.tEnd))/2).', ...
    [MBSim.system.nInputs,1]) .* uConst;

figure("Name", "System Inputs");
plot(MBSim.parameters.uSampleTimes, MBSim.parameters.uSampleValues);
grid on;
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("Inputs $u$", "Interpreter", "latex");
legend(arrayfun(@(x) sprintf("Input $u_%d$", x), 1:MBSim.system.nInputs), ...
    "Interpreter", "latex");

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-12;
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
