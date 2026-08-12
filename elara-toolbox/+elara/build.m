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

% Get target path (build folder)
targetDir = fullfile(elara.internal.getToolboxRootFolder, "+elara", "+mex");

%% Create output folder if not existing

if ~isfolder(targetDir)
    mkdir(targetDir);
end

%% Compile functions

fprintf("Compiling MEX functions...\n\n");

functionNames = [
    "elara.internal.integration.integrateVIBroyden"
    "elara.dynamics.num.firstOrderRHS"
    "elara.dynamics.num.firstOrderMassMatrix"
    "elara.dynamics.num.firstOrderDerivative"
    "elara.internal.simulation.getResultsFromStateTrajectory"
    "elara.statics.num.residual"
    "elara.internal.simulation.computeEnergies"
    ];

outputNames = [
    "integrateVIBroyden"
    "firstOrderRHS"
    "firstOrderMassMatrix"
    "firstOrderDerivative"
    "getResultsFromStateTrajectory"
    "staticResidual"
    "computeEnergies"
    ];

for iFun = 1:numel(functionNames)
    fprintf("Compiling function %d/%d (%s)...\n", ...
        iFun, numel(functionNames), functionNames(iFun));
    codegen("-d", targetDir, "-o", ...
        fullfile(targetDir, outputNames(iFun) + "_mex"), ...
        "-config", cfg, functionNames(iFun));
end

fprintf("MEX function compilation finished.\n");
