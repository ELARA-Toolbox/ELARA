classdef MBSimIntegratorVarIntBroyden < MBSimIntegrator
    %% MBSimulation Integrator: Variational Integrator with Broyden Solver
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Time step (s)
        h                           (1,1) double = 2^-8;

        % Target value for the solver error margin
        errorMargin                 (1,1) double = 1e-8;

        % Solver error margin at which the simulation is cancelled
        errorMarginLimit            (1,1) double = 1e-8;

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
    properties(Constant)
        type = "varint";
    end

    methods
        function simRes = simulateSystem(obj, MBSim)
            %% Simulate MBSystem: Variational integrator, Broyden solver
            arguments
                obj     (1,1)
                MBSim   (1,1) MBSimulation
            end

            % Check if compiled mex files are available
            useMex = exist("integrateMBSDynamics_Broyden_mex", "file");

            if obj.showConsoleOutput
                fprintf('Starting integration (varint-broyden)...\n');
            end

            % Copy solver settings into object (needed for codegen
            % function)
            solverOptionsVI = varIntSolverConfig;
            solverOptionsVI.h                = obj.h;
            solverOptionsVI.errorMargin      = obj.errorMargin;
            solverOptionsVI.errorMarginLimit = obj.errorMarginLimit;
            solverOptionsVI.maxIterations    = obj.maxIterations;
            solverOptionsVI.aTrapez          = obj.aTrapez;
            solverOptionsVI.JacobianIterationThreshold = obj.JacobianIterationThreshold;

            tic;
            if useMex
                simRes = integrateMBSDynamics_Broyden_mex(MBSim.MBSys, MBSim.simPars, solverOptionsVI);
            else
                simRes = integrateMBSDynamics_Broyden(MBSim.MBSys, MBSim.simPars, solverOptionsVI);
            end
            tSim = toc;

            % Run simulation again with timeit (if desired) for more accurate times
            if obj.accurateTiming
                if useMex
                    timingFun = @() integrateMBSDynamics_Broyden_mex(MBSim.MBSys, MBSim.simPars, solverOptionsVI);
                else
                    timingFun = @() integrateMBSDynamics_Broyden(MBSim.MBSys, MBSim.simPars, solverOptionsVI);
                end
                tSim = timeit(timingFun);
            end

            % Compute some metadata values of the simulation
            simRes.metaDataSim.TotalTime = tSim;
            % obj.simRes.getSimMetaData();

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
                    mean(simRes.solverError(:), 'omitnan'), ...
                    min (simRes.solverError(:)), ...
                    max (simRes.solverError(:)));
                fprintf('   Target solver error violations:      %d (%.2f%% of overall steps)\n', ...
                    sum(simRes.solverExitFlag == 1), sum(simRes.solverExitFlag == 1) / length(simRes.tout) *100);
                %fprintf('     Simulation exit code:           %d\n', exitCode);
            end
        end
    end
end