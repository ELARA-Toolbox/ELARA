classdef VIBroyden < elara.abstract.Integrator
    %% elara.Simulation Integrator: Variational Integrator with Broyden Solver
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Time step (s)
        h                           (1,1) double = 2^-8;

        % Target value for the solver error margin
        tolerance                   (1,1) double = 1e-8;

        % Solver error margin at which the simulation is cancelled
        toleranceLimit              (1,1) double = 1e-8;

        % Max. nr. of iterations of the implicit solver
        maxIterations               (1,1) double = 100;

        % For Broyden integrator:
        % Nr. of iterations that are allowed in one time step before the
        % Jacobian matrix is recomputed
        JacobianIterationThreshold  (1,1) double = 4;

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
                obj     (1,1)
                MBSim   (1,1) elara.Simulation
            end

            % Check if compiled mex files are available
            useMex = exist("integrateVIBroyden_mex", "file") == 3;

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
                simRes = integrateVIBroyden_mex( ...
                    MBSim.system, MBSim.parameters, solverOptionsVI);
            else
                simRes = elara.internal.integration.integrateVIBroyden( ...
                    MBSim.system, MBSim.parameters, solverOptionsVI);
            end
            tSim = toc;

            % Run simulation again with timeit (if desired) for more accurate times
            if obj.accurateTiming
                if useMex
                    timingFun = @() integrateVIBroyden_mex( ...
                        MBSim.system, MBSim.parameters, solverOptionsVI);
                else
                    timingFun = @() elara.internal.integration.integrateVIBroyden( ...
                        MBSim.system, MBSim.parameters, solverOptionsVI);
                end
                tSim = timeit(timingFun);
            end

            % Compute some metadata values of the simulation
            simRes.computationTime = tSim;
            % obj.results.getSimMetaData();

            if obj.showConsoleOutput
                % Display meta data
                if obj.accurateTiming
                    fprintf('   Total integration time (timeit):     %f s\n', tSim);
                else
                    fprintf('   Total integration time (tictoc):     %f s\n', tSim);
                end
                fprintf('   Simulation end time:                 %.3f s\n', simRes.tout(end));
                fprintf('   Nr. of successful time steps:        %d\n', length(simRes.tout));
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
                %fprintf('     Simulation exit code:           %d\n', exitCode);
            end
        end

        function fhs = plotSolverStats(~, simulation)
            arguments
                ~
                simulation (1,1) elara.Simulation
            end
            fhs = elara.plot.solverStatsVI(simulation.results, ...
                "nameString", simulation.Name);
        end
    end
end
