%% Generate Videos of the Examples
% The script runs the example scripts and then generates nicely formatted
% videos of each result.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

%% PD-Controlled Rigid-Flexible Robot

simulation_rigid_flexible_robot_PD_control;
close all;

%%
sim = MBSimODE;

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.5, 0.35];
ax.YLim = [-0.4,  0.35];
ax.ZLim = [-0.01, 1.55];
ax.View = [5, 20];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, links, ...
    "showInertialFrame", false, ...
    "showLinkFrames", false, ...
    "showJoints", true, ...
    "linkColorMap", colorMapFun);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_1", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);


%% Rigid-Flexible System

simulation_rigid_flexible_system;
close all

%%
sim = MBSimODE;

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.7, 1.6];
ax.YLim = [-0.2,  0.2];
ax.ZLim = [-1.5, 0.4];
ax.View = [35, 15];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, links, ...
    "showInertialFrame", false, ...
    "showLinkFrames", false, ...
    "showJoints", true, ...
    "linkColorMap", colorMapFun);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_2", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);


%% Continuum Manipulator (Forward Simulation)

simulation_continuum_manipulator;
close all

%%
sim = MBSimVI;

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.4, 0.15];
ax.YLim = [-0.05,  0.6];
ax.ZLim = [-0.05, 0.8];
ax.View = [-50, 30];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, links, ...
    "showInertialFrame", false, ...
    "showLinkFrames", false, ...
    "showJoints", true, ...
    "linkColorMap", colorMapFun);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_3", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);


%% Optimal Control Rigid Robot

optimal_control_rigid_robot;
close all;

%%
sim = MBSimOCP;

% Compute TCP trajectory
g_TCP_sol = zeros(4,4, OCP.nSteps+1);
for iStep = 1:OCP.nSteps+1
    g_k = OCP.systemNum.computeFwdKin(q_sol(:,iStep));
    g_TCP_sol(:,:,iStep) = g_k(:,:,OCP.systemNum.indexTCPFrame)*OCP.systemNum.g_B_TCP;
end
x_TCP_sol = squeeze(g_TCP_sol(1:3,4,:));

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.3, 0.8];
ax.YLim = [-0.2, 0.3];
ax.ZLim = [-0.02, 1.3];
ax.View = [55, 10];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, links, ...
    "showInertialFrame", false, ...
    "showLinkFrames", false, ...
    "showJoints", true, ...
    "linkColorMap", colorMapFun);

% Visualize workspace
OCP.workspace.visualize("createFigure", false);

% Plot TCP trajectory
plot3(OCP.x_TCP_traj(1,:), OCP.x_TCP_traj(2,:), OCP.x_TCP_traj(3,:), ...
    "--", "LineWidth", 2, "Color", "black");
plot3(x_TCP_sol(1,:), x_TCP_sol(2,:), x_TCP_sol(3,:), "-", "LineWidth", 2);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_4", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);


%% Optimal Control Planar Two-Link Robot

optimal_control_rigid_robot;
close all;

%%
sim = MBSimOCP;

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.5, 1.1];
ax.YLim = [-0.1, 0.1];
ax.ZLim = [-2.1, 2.1];
% ax.View = [55, 10];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, links, ...
    "showInertialFrame", false, ...
    "showLinkFrames", true, ...
    "showJoints", true, ...
    "linkColorMap", colorMapFun);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_5", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);


%% Optimal Control Continuum Manipulator
optimal_control_continuum_manipulator;
close all;

%%
sim = MBSimOCP;

% Compute TCP trajectory
g_TCP_sol = zeros(4,4, OCP.nSteps+1);
for iStep = 1:OCP.nSteps+1
    g_k = OCP.systemNum.computeFwdKin(q_sol(:,iStep));
    g_TCP_sol(:,:,iStep) = g_k(:,:,OCP.systemNum.indexTCPFrame)*OCP.systemNum.g_B_TCP;
end
x_TCP_sol = squeeze(g_TCP_sol(1:3,4,:));

% Create figure and axes
% Use tiledlayout with tight padding to maximize plot size
fh = figure("Theme", "Light");
tl = tiledlayout(fh, 1, 1, "Padding", "tight");
nexttile(tl);
[fh, ax] = elara.visualization.initializeAxes("createFigure", false);

fh.WindowState = "maximized";
fh.Color = [230,230,230]/255;

ax.Visible = "off";
ax.Projection = "perspective";
ax.XLim = [-0.4, 0.1];
ax.YLim = [-0.2, 0.1];
ax.ZLim = [-0.02, 0.8];
ax.View = [-45, 30];

% Remove axis labels and ticks to maximize effective plot size
ax.XLabel.Visible = "off";
ax.YLabel.Visible = "off";
ax.ZLabel.Visible = "off";
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

% Plot TCP trajectory
plot3(x_TCP_sol(1,:), x_TCP_sol(2,:), x_TCP_sol(3,:), "--", "LineWidth", 2);
plot3(x_TCP_sol(1,end), x_TCP_sol(2,end), x_TCP_sol(3,end), ".", "MarkerSize", 30);
plot3(x_TCP_sol(1,1), x_TCP_sol(2,1), x_TCP_sol(3,1), ".", "MarkerSize", 30);

%colorMapFun = @(x) nebula(x+1);
vis = elara.visualization.SystemVisualization(sim.system, link, ...
    "showInertialFrame", false, ...
    "showLinkFrames", false, ...
    "showJoints", true);%, ...
%"linkColorMap", colorMapFun);

vis.animateSystem(sim.results, fh, ...
    "saveMovie", true, ...
    "fileName", "example_video_6", ...
    "frameRate", 30, ...
    "displayTime", false, ...
    "updateAxisLimits", false);

