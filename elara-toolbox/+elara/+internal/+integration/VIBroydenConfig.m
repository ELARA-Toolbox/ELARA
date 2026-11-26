classdef VIBroydenConfig
    % Class to all settings for the variational integrator solvers

    properties
        % Time step (s)
        h                           (1,1) double {mustBePositive, mustBeFinite} = 2^-8;

        % Positive, finite target value for the solver error margin
        tolerance                   (1,1) double {mustBePositive, mustBeFinite} = 1e-8;

        % Accepted solver error limit; must be at least tolerance
        toleranceLimit              (1,1) double {mustBePositive, mustBeFinite} = 1e-8;

        % Maximum number of implicit-solver iterations
        maxIterations               (1,1) double {mustBeNonnegative, mustBeFinite, mustBeInteger} = 100;

        % For Broyden integrator:
        % Number of iterations allowed in one time step before the
        % Jacobian matrix is recomputed
        JacobianIterationThreshold  (1,1) double {mustBeNonnegative, mustBeFinite, mustBeInteger} = 4;

        % Generalized trapezoidal rule factor in [0, 1] for the interior
        % integration steps (only relevant for dissipation)
        % 0 = Rectangle Rule     (Second order only without dissipation)
        % 1/2 = Trapezoidal rule (Always second order)
        aTrapez                     (1,1) double {mustBeFinite, mustBeInRange(aTrapez, 0, 1)} = 0;
    end
end
