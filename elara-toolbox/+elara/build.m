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
targetDir = fullfile(elara.internal.getToolboxRootFolder, "build");

%% Compile functions

fprintf("Compiling MEX functions...\n\n");

functionNames = [
    "integrateMBSDynamics_Broyden"
    "elara.dynamics.num.firstOrderRHS"
    "elara.dynamics.num.firstOrderMassMatrix"
    "elara.dynamics.num.firstOrderDerivative"
    "getSimResFromStateTrajectory"
    "elara.statics.num.residual"
    "computeSimulationEnergies"
    ];

outputNames = [
    "integrateMBSDynamics_Broyden"
    "firstOrderRHS"
    "firstOrderMassMatrix"
    "firstOrderDerivative"
    "getSimResFromStateTrajectory"
    "staticResidual"
    "computeSimulationEnergies"
    ];

for iFun = 1:numel(functionNames)
    fprintf("Compiling function %d/%d (%s)...\n", ...
        iFun, numel(functionNames), functionNames(iFun));
    codegen("-d", targetDir, "-o", ...
        fullfile(targetDir, outputNames(iFun) + "_mex"), ...
        "-config", cfg, functionNames(iFun));
end

fprintf("MEX function compilation finished.\n");
