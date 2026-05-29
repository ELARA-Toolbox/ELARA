%% Build .mex files for important toolbox functions

%% Code Generation Settings
% Store in configuration object of class 'coder.MexCodeConfig'.

cfg = coder.config('mex');
cfg.TargetLang = 'C++';
cfg.GenerateReport = false;

% Function inlining "always" seems improve runtime significantly
cfg.InlineBetweenMathWorksFunctions = 'Always';
cfg.InlineBetweenUserAndMathWorksFunctions = 'Always';
cfg.InlineBetweenUserFunctions = 'Always';

% Disable memory checks etc. for better runtime performance
cfg.IntegrityChecks = false;
cfg.ResponsivenessChecks = true;


%% Execute codegen

% Variational integrator
codegen -d elara-toolbox/build  -config cfg integrateMBSDynamics_Broyden

% ODE functions
codegen -d elara-toolbox/build  -config cfg computeFirstOrderSystemRHS
codegen -d elara-toolbox/build  -config cfg computeFirstOrderMassMatrix
codegen -d elara-toolbox/build  -config cfg computeFirstOrderSystemRHS_MInv
codegen -d elara-toolbox/build  -config cfg getSimResFromStateTrajectory

% Static model
codegen -d elara-toolbox/build  -config cfg computeStaticResiduum

% Helper functions
codegen -d elara-toolbox/build -config cfg computeSimResEnergies


