%% Package ELARA Toolbox


%% Specify properties

identifier = "5848c156-85fa-4ef4-a9e0-5008f0eccc84";
toolboxFolder = elara.internal.getToolboxRootFolder;
repoFolder = fileparts(toolboxFolder);

opts = matlab.addons.toolbox.ToolboxOptions(toolboxFolder, identifier);

opts.ToolboxVersion = "0.2.1";
opts.ToolboxName = "ELARA Robot Simulation and Control Toolbox";
opts.ToolboxImageFile = fullfile(repoFolder, ...
    "doc", ".assets/elara_logo.png");
opts.ToolboxGettingStartedGuide = fullfile( toolboxFolder, ...
    "doc", "GettingStarted.mlx");

opts.Summary = "Efficient Lie-group Algorithms for Flexible Robotic Analysis and Control";
opts.Description = ...
    "ELARA is a MATLAB toolbox for the efficient simulation and optimal control of flexible robots.";

opts.AuthorName    = "Maximilian Herrmann";
opts.AuthorEmail   = "maximilian.herrmann@tum.de";
opts.AuthorCompany = "Technical University of Munich";

opts.SupportedPlatforms.Win64   = true;
opts.SupportedPlatforms.Maci64  = true;
opts.SupportedPlatforms.Glnxa64 = true;
opts.SupportedPlatforms.MatlabOnline = true;

opts.MinimumMatlabRelease = "R2025b";
opts.MaximumMatlabRelease = "";

opts.OutputFile = "elara-toolbox.mltbx";

%% Package Toolbox
matlab.addons.toolbox.packageToolbox(opts);