classdef SimulationResults
    % Class to store all simulation results of a multibody simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %% Simulation Results Data
        eta   (6,:,:)   double  % se(3) frame velocities   (6, nFrames, nTimes)
        g     (4,4,:,:) double  % Frame configurations     (4, 4, nFrames, nTimes)
        q     (:,:)     double  % Generalized coordinates  (nDoF, nTimes)
        q_dot (:,:)     double  % Generalized velocities   (nDoF, nTimes)
        tout  (:,1)     double  % Time values              (nTimes, 1)


        %% System Energies corresponding to the Simulation Results
        kineticEnergy   (:,1) double      % Kinetic energy
        potentialEnergy (:,1) double      % Potential energy (due to gravity)
        strainEnergy    (:,1) double      % Strain energy
        totalEnergy     (:,1) double      % Total energy (T+U+V)

        %% Solver Metadata for Implicit Solvers

        % Number of implicit-solver iterations (1, nTimes)
        solverIterations (1,:) double

        % Residual error after implicit-solver convergence (1, nTimes)
        solverResidual   (1,:) double

        % Exit flag of the implicit solver (Note: Meaning can depend on the
        % solver used) (1, nTimes)
        solverExitFlag   (1,:) double

        %% Overall Simulation Metadata

        % Total computational time of the simulation
        computationTime (1,1) double
    end
    methods (Static)
        function simRes = fromStateTrajectory(system, tout, q, q_dot, opts)
            %% Create simulation results from configuration and velocity trajectories
            % For example, use this method to post-process ODE integration
            % results.
            % The velocity matrix is optional; if no velocities are given,
            % they are computed from the configurations via finite
            % differences.
            arguments
                % Object defining the multibody system
                system  (1,1) elara.SystemNum

                % Time vector
                tout    (:,1) double

                % Configurations at the times in tout, (nDoF, nTimes)
                q       (:,:) double

                % Generalized velocities at the times in tout,
                % (nDoF, nTimes)
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
