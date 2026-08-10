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
    methods (Static)
        function simRes = fromStateTrajectory(system, tout, q, q_dot, opts)
            %% Get simRes object from trajectory of states q, q_dot
            % E.g., to post-process integration results from ODE
            % integrator.
            % The velocity matrix is optional; if no velocities are given,
            % they are computed from the configurations via finite
            % differences.
            arguments
                % Object defining the multibody system
                system  (1,1) elara.SystemNum

                % Time vector
                tout    (:,1) double

                % Matrix of configurations at time steps tout,
                % dimensions (nDoF, nSteps)
                q       (:,:) double

                % Matrix of velocities at time steps tout
                % dimensions (nDoF, nSteps)
                % Optional: if not given, they are computed via finite
                % differences.
                q_dot   (:,:) double = nan;

                % Order of the finite differences for the computation of
                % the velocities
                opts.finiteDifferenceOrder (1,1) double {mustBeMember(opts.finiteDifferenceOrder, [2,4])}= 2;
            end
            % Use finite differences if velocities are not given
            if isnan(q_dot)
                [q_dot, ~] = diffHigherOrder(q, tout(2)-tout(1), opts.finiteDifferenceOrder);
            end

            % Check if compiled mex files are available
            useMex = elara.internal.isMexAvailable("elara.mex.getResultsFromStateTrajectory_mex");

            if useMex
                simRes = elara.mex.getResultsFromStateTrajectory_mex( ...
                    system, tout, q, q_dot);
            else
                simRes = elara.internal.simulation.getResultsFromStateTrajectory( ...
                    system, tout, q, q_dot);
            end
        end
    end
end
