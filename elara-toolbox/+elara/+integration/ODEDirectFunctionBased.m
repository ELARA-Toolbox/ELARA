classdef ODEDirectFunctionBased < elara.abstract.IntegratorODE
    %% elara.Simulation Integrator: ODE solver
    % based on the direct/explicit form of the equations of motion, where
    % all EOM terms (including mass matrix) are explicitly computed;
    % implementation is based on the odeXY functions approach
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % ode function handle
        solverFunction  (1,1) function_handle = @ode45;

        % Solver options for the ode function
        solverOptions   (1,1) = odeset();
    end
    properties(Constant)
        type = "ode";
    end
    methods
        function simRes = simulateSystem(obj, MBSim)
            %% Simulate system: First order mass-matrix form / continuous-time ode solver
            arguments
                obj     (1,1)
                MBSim   (1,1) elara.Simulation
            end

            % Check if compiled mex files are available
            useMex = exist("firstOrderRHS_mex", "file");

            % Initial states for the first-order system
            x0 = [
                MBSim.parameters.q0;
                MBSim.parameters.qDot0;
                ];

            if obj.useMassMatrixForm
                if useMex
                    odeFun  = @(t,x) firstOrderRHS_mex(t, x, MBSim.system, MBSim.parameters);
                    massFun = @(t,x) firstOrderMassMatrix_mex(t, x, MBSim.system);
                else
                    odeFun  = @(t,x) elara.dynamics.num.firstOrderRHS(t, x, MBSim.system, MBSim.parameters);
                    massFun = @(t,x) elara.dynamics.num.firstOrderMassMatrix(t, x, MBSim.system);
                end
                odeOpts = odeset(obj.solverOptions, "Mass", massFun);
            else
                if useMex
                    odeFun = @(t,x) firstOrderDerivative_mex(t, x, MBSim.system, MBSim.parameters);
                else
                    odeFun = @(t,x) elara.dynamics.num.firstOrderDerivative(t, x, MBSim.system, MBSim.parameters);
                end
                odeOpts = obj.solverOptions;
            end

            if obj.showConsoleOutput
                fprintf("Starting integration (ode-direct function-based, solver %s)...\n", ...
                    functions(obj.solverFunction).function);
            end
            tic;
            [tout, xout] = obj.solverFunction(odeFun, [0, MBSim.parameters.tEnd], x0, odeOpts);
            tSim = toc;

            % Run simulation again with timing (if desired) for accuracy
            if obj.accurateTiming
                timingFun = @() obj.solverFunction(odeFun, [0, MBSim.parameters.tEnd], x0, odeOpts);
                tSim = timeit(timingFun, 2);
            end

            % ODE post-processing
            if obj.showConsoleOutput
                disp("Postprocessing integration results...")
            end

            simRes = elara.SimulationResults.fromStateTrajectory( MBSim.system, tout, ...
                xout(:,1:MBSim.system.nDoF).', xout(:,(MBSim.system.nDoF+1):end).'...
                );
            tFullODE = toc;
            simRes.computationTime = tSim;

            if obj.showConsoleOutput
                % Display meta data
                tStepODE = diff(tout);
                if isfield(obj.solverOptions, "AbsTol")
                    absTol = obj.solverOptions.AbsTol;
                else
                    absTol = nan;
                end
                if isfield(obj.solverOptions, "RelTol")
                    relTol = obj.solverOptions.RelTol;
                else
                    relTol = nan;
                end

                if obj.showConsoleOutput
                    fprintf('   Total integration time (timeit):     %f s\n', tSim);
                else
                    fprintf('   Total integration time (tictoc):     %f s\n', tSim);
                end
                fprintf('   Simulation end time:                 %.3f s\n', tout(end));
                fprintf('   Nr. of time steps:                   %d\n', numel(tout))
                fprintf('   Time step (Avg/Min/Max):             %.3e / %.3e / %.3e s\n', ...
                    mean(tStepODE, 'omitnan'), min(tStepODE), max(tStepODE));
                fprintf('   Approx. comp. time per step:         %.3e s\n', tSim / numel(tout))
                fprintf('   Solver tolerances (Abs/Rel):         %.3e / %.3e\n', absTol, relTol);
                fprintf('   Total time incl. postprocessing:     %.4f s\n', tFullODE)
            end
        end
    end
end
