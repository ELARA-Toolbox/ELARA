%% Simulation Example for the 2025 Chiemsee pHS Spring School Lecture Notes
%
% The script simulates the motion of a rigid-flexible multibody system:
% the system is a precurved cantilever beam with an aluminium plate
% mounted at its end via a revolute joint.
% The simulation starts in the equilibrium configuration, and the system is
% excited by an external force at the start.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Define System

% Compute reference deformation so that the unstressed beam describes a
% quarter circle
L = 0.75;          % Beam Length
xiBend = 2*pi/L/4; % Deformation (rotational x component)
radius = L/4/pi*8; % Resulting radius

%%% Define Link Objects

% Link 1: Cantilever link
links(1) = elara.FlexibleLink;
links(1).parentLink = 0;
links(1).isCantilever = true;
links(1).nSegments = 8;
links(1).L         = L;
links(1).g_J_B     = eye(4);
links(1).Ba = [ eye(3); zeros(3)];
links(1).Bc = [ zeros(3); eye(3)];
links(1).xiRef = repmat([xiBend;0;0;0;0;1], [1,links(1).nSegments]);
links(1).beamParameters = beamParams_ASA_round("radius",0.01);
links(1).beamParameters.d = ones(6,1)*1e-4*0;

% Link 2: Rigid link (solid cuboid)
links(2) = elara.RigidLink;

cX = 0.3;  % Height
cY = 0.2;  % Length
cZ = 0.02; % Width
cRho = 2.85e3;       % Density
cM = cX*cY*cZ*cRho;  % Mass
cJ = cM/12 * diag([cY^2 + cZ^2, cX^2 + cZ^2, cX^2 + cY^2]); % Inertia tensor

links(2).parentLink = 1;
links(2).jointIsActuated = 1;
links(2).jointAxis  = [0 0 1 0 0 0].';
links(2).g_J_B      = SE3Matrix(eye(3), [cX/2-0.02,-cY/2+0.02,0]);
links(2).g_ref      = SE3Matrix(eye(3), [cX/2-0.02,-cY/2+0.02,0]);
links(2).m          = cM;
links(2).J          = cJ;
links(2).d          = 0*1e-4;

% Add bounding box centered at COM
links(2).g_bbox = eye(4);
links(2).bBoxSize = [
    +cX, +cY, +cZ
    -cX, -cY, -cZ
    ]/2;


%% Initialize Simulation object

MBSim = elara.Simulation(links, "displayInfo", true);

% Specify pose of the first beam frame: Beam lies in the XY-plane and is
% aligned with global x axis
RRef0 = [0 0 1; 0 1 0; -1 0 0];
MBSim.system.g0 = SE3Matrix(RRef0, zeros(3,1));


%% Visualize reference configuration

qRef = MBSim.system.setLinkDeformations(links(1).xiRef, 1);
MBSim.visualizeSystemConfig(qRef);

% Add circle to illustrate radius
th = 0:pi/100:2*pi;
xunit = radius * cos(th) + radius;
yunit = radius * sin(th) + 0;
h = plot(xunit-radius, yunit-radius);

gRef = MBSim.system.computeFwdKin(qRef);
plot3(gRef(1,4,end-1),gRef(2,4,end-1),gRef(3,4,end-1),'x','MarkerSize', 20)

disp('Computed circle radius:')
disp(radius)
disp('Beam end node positions:')
disp(gRef(1:3,4,end-1).')


%% Compute equilibrium configuration

COMPUTE_EQU_CONFIG = true;

if COMPUTE_EQU_CONFIG
    uEqu = zeros(MBSim.system.nInputs, 1);
    tic;
    qEqu = fsolve( ...
        @(q) computeStaticResiduum(MBSim.system, MBSim.parameters, q, uEqu), ...
        zeros(MBSim.system.nDoF,1) ...
        );
    tEquFsolve = toc;

    MBSim.visualizeSystemConfig(qEqu, "figureName", "VisEqu");
    title("Equilibrium configuration for u = 0");

    q0 = qEqu;
else
    q0 = qRef;
end


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 10;

% Initial configuration
MBSim.parameters.q0 = q0;
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);


%% External frame forces

% Add spatial external wrench with smooth impulse at the beginning
maximumWrench = zeros(6,MBSim.system.nFrames);
maximumWrench(:,end-1) = [0;0;0;0;0;30];
MBSim.parameters.externalWrench_s = MBSim.parameters.externalWrench_s.addWrench( ...
    0, 1, 4, maximumWrench); % Smooth impulse


% Plot forces
tVec = 0:0.005:MBSim.parameters.tEnd;
extFrameForce_b = zeros(6,MBSim.system.nFrames,length(tVec));
extFrameForce_s = zeros(6,MBSim.system.nFrames,length(tVec));
for iStep = 1:length(tVec)
    extFrameForce_b(:,:,iStep) = MBSim.parameters.externalWrench_b.getCurrentWrench( ...
        MBSim.system.nFrames, tVec(iStep));
    extFrameForce_s(:,:,iStep) = MBSim.parameters.externalWrench_s.getCurrentWrench( ...
        MBSim.system.nFrames, tVec(iStep));
end

figure("Name","externalForces", "NumberTitle", "off");
tiledlayout("vertical");
nexttile;
plot(tVec, squeeze(extFrameForce_b(:,end-1,:)));
title(sprintf("Body-Fixed Wrench over time (frame %d)", MBSim.system.nFrames-1));
legend(arrayfun(@(x) sprintf("$f_%d$", x), 1:6), "Interpreter", "latex");
grid on;
nexttile;
plot(tVec, squeeze(extFrameForce_s(:,end-1,:)));
title(sprintf("Spatial Wrench over time (frame %d)", MBSim.system.nFrames-1));
legend(arrayfun(@(x) sprintf("$f_%d$", x), 1:6), "Interpreter", "latex");
grid on;

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 1e-4;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 1e-11;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting results
MBSimVI.plotAll;

% Compute energies
MBSimVI = MBSimVI.computeEnergies;

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Plots

%%% Reference config
[fh,vis]  = MBSim.visualizeSystemConfig(qRef, "figureName", "plotRefConf", ...
    "ShowInertialFrame", false);
xlim([0, 0.65]);
ylim([-0.55, 0.1]);
zlim([-0.55, 0.1]);
xlabel('$x$ / m','Interpreter','latex');
ylabel('$y$ / m','Interpreter','latex');
zlabel('$z$ / m','Interpreter','latex');
fh.WindowStyle = "normal";
fh.CurrentAxes.TickLabelInterpreter = "latex";
fh.Theme = "Light";
view(-37.5,30);

% 2D Projections
drawSystemVisProjection(vis, "yz", fh.CurrentAxes.XLim(2));
drawSystemVisProjection(vis, "xz", fh.CurrentAxes.YLim(2));


%%% Initial config
[fh,vis] = MBSim.visualizeSystemConfig(q0, "figureName", "plotInitConf", ...
    "ShowInertialFrame", false);
xlim([0, 0.65]);
ylim([-0.55, 0.1]);
zlim([-0.55, 0.1]);
xlabel('$x$ / m','Interpreter','latex');
ylabel('$y$ / m','Interpreter','latex');
zlabel('$z$ / m','Interpreter','latex');
fh.WindowStyle = "normal";
fh.CurrentAxes.TickLabelInterpreter = "latex";
fh.Theme = "Light";
view(-37.5,30);

% 2D Projections
drawSystemVisProjection(vis, "yz", fh.CurrentAxes.XLim(2));
drawSystemVisProjection(vis, "xz", fh.CurrentAxes.YLim(2));


%%% Snapshots
fh = MBSimVI.drawSnapshots("nSnapShots", 10);
fh.WindowStyle = "normal";
fh.Name = "plotSnapshots";
fh.NumberTitle = "off";
xlim([0, 0.8]);
ylim([-0.65, 0.1]);
zlim([-0.65, 0.1]);
xlabel('$x$ / m','Interpreter','latex');
ylabel('$y$ / m','Interpreter','latex');
zlabel('$z$ / m','Interpreter','latex');
fh.WindowStyle = "normal";
fh.CurrentAxes.TickLabelInterpreter = "latex";
fh.Theme = "Light";
view(-37.5,30);
title("");


%%% Energies: All energies
fh = figure('Name', 'plotEnergies', 'NumberTitle','off');
fh.WindowStyle = "normal";
ax = axes(fh);

plot(ax, MBSimVI.results.tout, MBSimVI.results.totalEnergy, 'LineWidth', 1.5);
hold on;
plot(ax, MBSimVI.results.tout, MBSimVI.results.kineticEnergy, 'LineWidth', 1.2);
plot(ax, MBSimVI.results.tout, MBSimVI.results.potentialEnergy, 'LineWidth', 1.2);
plot(ax, MBSimVI.results.tout, MBSimVI.results.strainEnergy, 'LineWidth', 1.2);
grid on
xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')
ylabel(ax, 'energy / J', 'interpreter', 'latex')
legend(ax, 'total', 'kinetic', 'potential', 'strain', ...
    'interpreter', 'latex', 'Location', 'southeast');
fh.CurrentAxes.TickLabelInterpreter = "latex";
fh.Theme = "Light";


%%% Energies: Total energy after forcing
fh = figure('Name', 'plotEnergiesZoom', 'NumberTitle','off');
fh.WindowStyle = "normal";
ax = axes(fh);

plot(ax, MBSimVI.results.tout, MBSimVI.results.totalEnergy, 'LineWidth', 1.5);
grid on
xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')
ylabel(ax, 'total energy $H$ / J', 'interpreter', 'latex')
fh.Theme = "Light";
fh.CurrentAxes.TickLabelInterpreter = "latex";

% Cheap workaround to automatically get the y axis limits for the constant
% part
xlim([1,10])
ylims = fh.CurrentAxes.YLim;
xlim([0,10]);
ylim(ylims);


%%% Joint Angle
fh = figure('Name', 'plotJointAngle', 'NumberTitle','off');
fh.WindowStyle = "normal";
plot(MBSimVI.results.tout, rad2deg(MBSimVI.results.q(end,:)), 'LineWidth', 1.2);
grid on
xlabel('time $t$ / s', 'interpreter', 'latex')
ylabel('$\theta$ / deg', 'interpreter', 'latex')
fh.Theme = "Light";
fh.CurrentAxes.TickLabelInterpreter = "latex";


%%% Strains psi-psiRef of one segment
iLink = 1;
xi = zeros(6,MBSimVI.links(iLink).nSegments, length(MBSimVI.results.tout));
for iStep = 1:length(MBSimVI.results.tout)
    xi(:,:,iStep) = MBSimVI.system.getLinkDeformations(MBSimVI.results.q(:,iStep), iLink);
end

fh = figure('Name', 'plotStrains', 'NumberTitle','off');
fh.Theme = "Light";
fh.WindowStyle = "normal";

iSeg = 5; % Segment to plot
plot(MBSimVI.results.tout, ...
    squeeze(xi(1:3,iSeg,:))-repmat(links(1).xiRef(1:3, iSeg), [1, length(MBSimVI.results.tout)]), ...
    'LineWidth', 1.2);
grid on
xlabel('time $t$ / s', 'interpreter', 'latex')
ylabel(sprintf("$(\\psi_%d-\\bar{\\psi}_%d)$", iSeg,iSeg), 'interpreter', 'latex')
fh.CurrentAxes.TickLabelInterpreter = "latex";
legend("bending $x$", "bending $y$", "torsion", "Interpreter", "latex");


%% End script
disp("Finished.")
