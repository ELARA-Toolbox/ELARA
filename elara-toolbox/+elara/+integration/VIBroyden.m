classdef VIBroyden < elara.abstract.Integrator
    %% elara.Simulation Integrator: Variational Integrator with Broyden Solver
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Time step (s)
        h                           (1,1) double {mustBePositive, mustBeFinite} = 2^-8;

        % Positive, finite target value for the solver error margin
        tolerance                   (1,1) double {mustBePositive, mustBeFinite} = 1e-8;

        % Accepted solver error limit; raised to tolerance for the run if smaller
        toleranceLimit              (1,1) double {mustBePositive, mustBeFinite} = 1e-8;

        % Maximum number of implicit-solver iterations
        maxIterations               (1,1) double {mustBeNonnegative, mustBeFinite, mustBeInteger} = 100;

        % For Broyden integrator:
        % Number of iterations allowed in one time step before the
        % Jacobian matrix is recomputed
        JacobianIterationThreshold  (1,1) double {mustBeNonnegative, mustBeFinite, mustBeInteger} = 4;

        % Whether to use first- or second-order dissipation term.
        % (first order: more robust; second order: more accurate)
        %
        % This sets the generalized trapezoidal rule factor a of the interior
        % integration steps. It is only relevant if dissipation is present.
        % true  -> a = 0   -> Rectangle Rule (Second order only without dissipation)
        % false -> a = 1/2 -> Trapezoidal rule (Always second order)
        useFirstOrderDissipation    (1,1) logical = 1;
    end
    properties(Constant)
        type = "varint";
    end

    methods
        function simRes = simulateSystem(obj, MBSim)
            %% Simulate ELARA System: Variational integrator, Broyden solver
            arguments
                obj     (1,1) elara.integration.VIBroyden
                MBSim   (1,1) elara.Simulation
            end

            if obj.toleranceLimit < obj.tolerance
                warning('elara:ToleranceLimitAdjusted', ...
                    ['toleranceLimit (%.3e) is smaller than tolerance (%.3e). ' ...
                    'Using toleranceLimit = tolerance for this run.'], ...
                    obj.toleranceLimit, obj.tolerance);
                obj.toleranceLimit = obj.tolerance;
            end

            % Check if compiled mex files are available
            useMex = elara.internal.isMexAvailable("elara.mex.integrateVIBroyden_mex");

            if obj.showConsoleOutput
                fprintf('Starting integration (varint-broyden)...\n');
            end

            % Copy solver settings into object (needed for codegen function)
            solverOptionsVI = elara.internal.integration.VIBroydenConfig;
            solverOptionsVI.h                = obj.h;
            solverOptionsVI.tolerance      = obj.tolerance;
            solverOptionsVI.toleranceLimit = obj.toleranceLimit;
            solverOptionsVI.maxIterations    = obj.maxIterations;
            solverOptionsVI.JacobianIterationThreshold = obj.JacobianIterationThreshold;

            % Set generalized trapezoidal rule factor for dissipation
            if obj.useFirstOrderDissipation
                solverOptionsVI.aTrapez = 0;
            else
                solverOptionsVI.aTrapez = 1/2;
            end

            tic;
            if useMex
                simRes = elara.mex.integrateVIBroyden_mex( ...
                    MBSim.system, MBSim.parameters, solverOptionsVI);
            else
                simRes = elara.internal.integration.integrateVIBroyden( ...
                    MBSim.system, MBSim.parameters, solverOptionsVI);
            end
            tSim = toc;

            simRes.simulationStatus = obj.assessSimulationStatus(simRes, ...
                round(MBSim.parameters.tEnd / obj.h));

            % Run simulation again with timeit (if desired) for more accurate times
            if obj.accurateTiming && simRes.simulationStatus ~= 0
                if useMex
                    timingFun = @() elara.mex.integrateVIBroyden_mex( ...
                        MBSim.system, MBSim.parameters, solverOptionsVI);
                else
                    timingFun = @() elara.internal.integration.integrateVIBroyden( ...
                        MBSim.system, MBSim.parameters, solverOptionsVI);
                end
                tSim = timeit(timingFun);
            end

            % Compute some metadata values of the simulation
            simRes.computationTime = tSim;

            if obj.showConsoleOutput
                % Display metadata
                if obj.accurateTiming && simRes.simulationStatus ~= 0
                    fprintf('   Total integration time (timeit):     %f s\n', tSim);
                else
                    fprintf('   Total integration time (tictoc):     %f s\n', tSim);
                end
                fprintf('   Simulation end time:                 %.3f s\n', simRes.tout(end));
                fprintf('   Number of successful time steps:     %d\n', length(simRes.tout)-1);
                fprintf('   Time step:                           %.3e s = 2^%.2f s\n', obj.h, log2(obj.h));
                fprintf('   Approx. comp. time per step:         %.3e s\n', tSim / length(simRes.tout))
                fprintf('   Iteration Count (Avg/Min/Max/Total): %.4f / %d / %d / %d\n', ...
                    mean(simRes.solverIterations, 'omitnan'), ...
                    min (simRes.solverIterations), ...
                    max (simRes.solverIterations), ...
                    sum(simRes.solverIterations, "omitmissing"));
                fprintf('   Residual (Avg/Min/Max):              %.4g / %.4g / %.4g\n', ...
                    mean(simRes.solverResidual(:), 'omitnan'), ...
                    min (simRes.solverResidual(:)), ...
                    max (simRes.solverResidual(:)));
                fprintf('   Target solver error violations:      %d (%.2f%% of overall steps)\n', ...
                    sum(simRes.solverExitFlag == 1), sum(simRes.solverExitFlag == 1) / length(simRes.tout) *100);
                fprintf('   Simulation status:                   %d\n', simRes.simulationStatus);
            end
        end

        function fhs = plotSolverStats(obj, simulation)
            arguments
                obj        (1,1) elara.integration.VIBroyden
                simulation (1,1) elara.Simulation
            end
            fhs = elara.plot.solverStatsVI(simulation.results, ...
                "nameString", simulation.Name);
        end
    end

    methods (Access = private)
        function status = assessSimulationStatus(obj, simRes, requestedSteps)
            % Assess completion and both tolerances after integration.
            arguments
                obj            (1,1) elara.integration.VIBroyden
                simRes         (1,1) elara.SimulationResults
                requestedSteps (1,1) double {mustBePositive, mustBeFinite, mustBeInteger}
            end

            % The final node has no outgoing step; its residual is NaN.
            stepResiduals = simRes.solverResidual(1:end-1);

            status = 1;
            if numel(simRes.tout) ~= requestedSteps + 1 ...
                    || any(~isfinite(simRes.q(:,end))) ...
                    || any(~isfinite(stepResiduals)) ...
                    || any(stepResiduals > obj.toleranceLimit) ...
                    || obj.tolerance > obj.toleranceLimit
                status = 0;
                warning('elara:IntegrationFailed', ...
                    'Variational integration failed. Returning available simulation results.');
            elseif any(stepResiduals > obj.tolerance)
                status = 2;
            end
        end
    end
end
