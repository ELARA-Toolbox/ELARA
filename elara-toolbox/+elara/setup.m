%% Check ELARA Toolbox installation

fprintf("Verifying ELARA toolbox installation...\n")

%% Check core toolbox installation

if exist("elara.Simulation", "class")
    fprintf("  [PASS]  Core toolbox installed correctly.\n")
else
    fprintf("  [FAIL]  Core toolbox not installed correctly. " + ...
        "Please verify your installation.")
end

%% Check for compiled mex files

mexFileNames = [
    "elara.mex.integrateVIBroyden_mex"
    "elara.mex.firstOrderRHS_mex"
    "elara.mex.firstOrderMassMatrix_mex"
    "elara.mex.firstOrderDerivative_mex"
    "elara.mex.getResultsFromStateTrajectory_mex"
    "elara.mex.staticResidual_mex"
    "elara.mex.computeEnergies_mex"
    ];
fileExists = false(size(mexFileNames));
for iFile = 1:numel(mexFileNames)
    fileExists(iFile) = elara.internal.isMexAvailable(mexFileNames(iFile));
end

if all(fileExists)
    fprintf("  [PASS]  Compiled MEX files found under elara.mex.\n")
elseif any(fileExists)
    fprintf("  [FAIL]  MEX installation is incomplete. Falling back to MATLAB where needed.\n")
    fprintf("          Missing: %s\n", strjoin(mexFileNames(~fileExists), ", "));
else
    fprintf("  [FAIL]  No compiled MEX files found under elara.mex. " + ...
        "Falling back to MATLAB functions.\n")
end

%% Check CasADi installation
if exist("casadi.MX", "class")
    fprintf("  [PASS]  CasADi installation found.\n")
else
    fprintf("  [FAIL]  No CasADi installation found on MATLAB path.")
end
