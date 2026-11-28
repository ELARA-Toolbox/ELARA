classdef SimulationResults
    % Class to store all simulation results of a multibody simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %% Simulation Results Data
        eta   (6,:,:)   double  % se(3) frame velocities   (6, nFrames, nSteps)
        g     (4,4,:,:) double  % Frame configurations     (4, 4, nFrames, nSteps)
        q     (:,:)     double  % Generalized coordinates  (nDoF, nSteps)
        q_dot (:,:)     double  % Generalized velocities   (nDoF, nSteps)
        tout  (:,1)     double  % Time values              (nSteps, 1)


        %% System Energies corresponding to the Simulation Results
        kineticEnergy   (:,1) double      % Kinetic energy
        potentialEnergy (:,1) double      % Potential energy (due to gravity)
        strainEnergy    (:,1) double      % Strain energy
        totalEnergy     (:,1) double      % Total energy (T+U+V)

        %% Solver Metadata for Implicit Solvers

        % Number of implicit-solver iterations (1, nSteps)
        solverIterations (1,:) double

        % Residual error after implicit-solver convergence (1, nSteps)
        solverResidual   (1,:) double

        % Exit flag of the implicit solver (Note: Meaning can depend on the
        % solver used) (1, nSteps)
        solverExitFlag   (1,:) double

        %% Overall Simulation Metadata

        % Overall status: 0 = failed or not yet simulated, 1 = successful,
        % 2 = successful using toleranceLimit instead of tolerance (VI only).
        simulationStatus (1,1) double = 0;

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
            % differences, which require uniformly spaced sample times and
            % at least four samples for order 2 or six samples for order 4.
            arguments
                % Object defining the multibody system
                system  (1,1) elara.SystemNum

                % Time vector; must be uniform when estimating velocities.
                tout    (:,1) double

                % Configurations at the times in tout, (nDoF, nSteps)
                q       (:,:) double

                % Generalized velocities at the times in tout,
                % (nDoF, nSteps)
                % Optional: if not given, they are computed via finite
                % differences.
                q_dot   (:,:) double = nan;

                % Order of the finite differences for the computation of
                % the velocities
                opts.finiteDifferenceOrder (1,1) double {mustBeMember(opts.finiteDifferenceOrder, [2,4])}= 2;
            end
            % Scalar NaN marks omitted velocities. Explicit velocity arrays
            % may contain NaNs, for example at the final VI time node.
            if isscalar(q_dot) && isnan(q_dot)
                assert(numel(tout) >= 2, 'elara:InvalidTimeGrid', ...
                    'Finite-difference velocities require at least two sample times.');
                timeSteps = diff(tout);
                h = mean(timeSteps);
                assert(isreal(tout) && all(isfinite(tout)) ...
                    && all(timeSteps > 0) && isfinite(h), ...
                    'elara:InvalidTimeGrid', ...
                    'Finite-difference velocities require finite, strictly increasing sample times.');

                % Allow small relative spacing differences and roundoff
                % from subtracting timestamps with a nonzero time origin.
                gridTolerance = 1e-8*abs(h) + 8*eps(max(abs(tout)));
                assert(all(abs(timeSteps-h) <= gridTolerance), ...
                    'elara:NonuniformTimeGrid', ...
                    ['Finite-difference velocities require uniformly spaced sample times. ' ...
                    'Supply q_dot explicitly for a nonuniform time grid.']);
                assert(numel(tout) >= opts.finiteDifferenceOrder + 2, ...
                    'elara:InsufficientSamples', ...
                    ['Finite-difference velocities require four samples for order 2 or six for order 4. ' ...
                    'Supply q_dot explicitly for a shorter trajectory.']);
                [q_dot, ~] = diffHigherOrder(q, h, opts.finiteDifferenceOrder);
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
