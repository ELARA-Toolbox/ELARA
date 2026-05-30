%% Build .mex files for important toolbox functions

%% Code Generation Settings

cfg = coder.config('mex');
cfg.TargetLang = 'C++';
cfg.GenerateReport = false;

% Function inlining "always" seems to improve runtime significantly
cfg.InlineBetweenMathWorksFunctions = 'Always';
cfg.InlineBetweenUserAndMathWorksFunctions = 'Always';
cfg.InlineBetweenUserFunctions = 'Always';

% Disable memory checks etc. for better runtime performance
cfg.IntegrityChecks = false;
cfg.ResponsivenessChecks = true;

%% Compile functions

% Get target path (build folder)
thisFile = mfilename("fullpath");
buildFolder = fileparts(thisFile);
toolboxRoot = fileparts(buildFolder);
targetDir = fullfile(toolboxRoot, "build");

fprintf("Compiling MEX functions...\n\n");

functionNames = [
    "integrateMBSDynamics_Broyden"
    "computeFirstOrderSystemRHS"
    "computeFirstOrderMassMatrix"
    "computeFirstOrderSystemRHS_MInv"
    "getSimResFromStateTrajectory"
    "computeStaticResiduum"
    "computeSimResEnergies"
    ];

parfor iFun = 1:numel(functionNames)
    codegen("-d", targetDir, "-config", cfg, functionNames(iFun));
end

fprintf("MEX function compilation finished.\n");
