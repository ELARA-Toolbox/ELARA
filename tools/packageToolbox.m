%% Package ELARA Toolbox


%% Specify properties

identifier = "5848c156-85fa-4ef4-a9e0-5008f0eccc84";
toolboxFolder = "elara-toolbox";

opts = matlab.addons.toolbox.ToolboxOptions(toolboxFolder,identifier);

opts.ToolboxVersion = "0.1";
opts.ToolboxName = "ELARA Robot Simulation and Control Toolbox";
opts.ToolboxImageFile = "doc/.assets/elara_logo.png";
opts.ToolboxGettingStartedGuide = "elara-toolbox/doc/GettingStarted.mlx";

opts.Summary = "Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control";
opts.Description = "";

opts.AuthorName    = "Maximilian Herrmann";
opts.AuthorEmail   = "maximilian.herrmann@tum.de";
opts.AuthorCompany = "Technical University of Munich";

opts.SupportedPlatforms.Win64   = true;
opts.SupportedPlatforms.Mac     = true;
opts.SupportedPlatforms.Glnxa64 = true;
opts.SupportedPlatforms.MatlabOnline = true;

opts.MinimumMatlabRelease = "R2025b";
opts.MaximumMatlabRelease = "";

opts.OutputFile = "elara-toolbox.mltbx";

%% Package Toolbox
matlab.addons.toolbox.packageToolbox(opts);