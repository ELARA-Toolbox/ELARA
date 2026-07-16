classdef varIntSolverConfig
    % Class to all settings for the variational integrator solvers

    properties
        % Time step (s)
        h                           (1,1) double = 2^-8;
        
        % Target value for the solver error margin
        tolerance                 (1,1) double = 1e-8;

        % Solver error margin at which the simulation is cancelled
        toleranceLimit            (1,1) double = 1e-8;

        % Max. nr. of iterations of the implicit solver
        maxIterations               (1,1) double = 100;

        % For Broyden integrator:
        % Nr. of iterations that are allowed in one time step before the
        % Jacobian matrix is recomputed
        JacobianIterationThreshold  (1,1) double = 4;

        % Generalized trapezoidal rule factor of the interior integration
        % steps (only relevant for dissipation)
        % 0 = Rectangle Rule     (Second order only without dissipation)
        % 1/2 = Trapezoidal rule (Always second order)
        aTrapez                     (1,1) double = 0;
    end
end
