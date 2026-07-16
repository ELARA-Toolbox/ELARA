classdef SimulationResults
    % Class to store all simulation results of a multibody simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    properties
        %% Simulation Results Data
        eta   (6,:,:)   double  % se(3)/ R6 node velocity vectors            (6,   nLinks, nSteps)
        g     (4,4,:,:) double  % link/node configuration matrices           (4,4, nLinks, nSteps)
        q     (:,:)     double  % Generalized coordinates                    (nDoF, nSteps)
        q_dot (:,:)     double  % Generalized velocities                     (6,   nJoints, nSteps)
        tout  (:,1)     double  % time values                                (nSteps, 1)


        %% System Energies corresponding to the Simulation Results
        kineticEnergy   (:,1) double      % Kinetic energy
        potentialEnergy (:,1) double      % Potential energy (due to gravity)
        strainEnergy    (:,1) double      % Strain energy
        totalEnergy     (:,1) double      % Total energy (T+U+V)

        %% Solver Meta Data for Implicit Solvers

        % Nr of iterations of the implicit solver (1, nSteps)
        solverIterations (1,:) double

        % Residual error of the implicit equations after convergence (1, nSteps)
        solverResidual   (1,:) double

        % Exit flag of the implicit solver (Note: Meaning can depend on the
        % used solver) (1, nSteps)
        solverExitFlag   (1,:) double

        %% Meta Data of the Overall Simulation

        % Total computational time of the simulation
        computationTime (1,1) double
    end
end