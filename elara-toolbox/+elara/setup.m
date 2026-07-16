%% Check ELARA Toolbox installation

fprintf("Verifying ELARA toolbox installation...\n")

%% Check core toolbox installation

if exist("elara.Simulation", "file")
    fprintf("  [PASS]  Core toolbox installed correctly.\n")
else
    fprintf("  [FAIL]  Core toolbox not installed correctly. " + ...
        "Please verify your installation.")
end

%% Check for compiled mexfiles

mexFileNames = [
    "integrateMBSDynamics_Broyden_mex"
    "computeFirstOrderSystemRHS_mex"
    "computeFirstOrderMassMatrix_mex"
    "computeFirstOrderSystemRHS_MInv_mex"
    "getSimResFromStateTrajectory_mex"
    "computeStaticResiduum_mex"
    "computeSimResEnergies_mex"
    ];
fileExists = zeros(size(mexFileNames));
for iFile = 1:numel(mexFileNames)
    fileExists(iFile) = exist(mexFileNames(iFile), "file");
end

if all(fileExists == 3)
    fprintf("  [PASS]  Compiled mex files found.\n")
else
    fprintf("  [FAIL]  No compiled mex files found. Falling back to standard MATLAB functions.\n")
end

%% Check CasADi installation
if exist("casadi.MX", "class")
    fprintf("  [PASS]  CasADi installation found.\n")
else
    fprintf("  [FAIL]  No CasADi installation found on MATLAB path.")
end