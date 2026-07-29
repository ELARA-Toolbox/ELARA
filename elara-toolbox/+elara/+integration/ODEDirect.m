classdef ODEDirect < elara.abstract.IntegratorODE
    %% elara.Simulation Integrator: ODE solver
    % based on the direct/explicit form of the equations of motion, where
    % all EOM terms (including mass matrix) are explicitly computed;
    % implementation is based on the object-based approach (with matlab ode
    % object)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % ODE object that is used for integration of the continuous-time
        % equations of motion;
        % all necessary settings (solver selection, tolerances etc.) must
        % be done to this object before the simulateSystem() method is
        % called
        odeObject   (1,1) ode = ode();

    end
    properties(Constant)
        type = "ode";
    end
    methods
        function simRes = simulateSystem(obj, MBSim)
            %% Simulate system: First order mass-matrix form / continuous-time ode solver
            % Note: It is assumed that all additional solver options,
            % including tolerances, have already been set on the
            % odeObject before
            arguments
                obj     (1,1)
                MBSim   (1,1) elara.Simulation
            end

            % Check if compiled mex files are available
            useMex = exist("computeFirstOrderSystemRHS_mex", "file");

            % Initial states for the first-order system
            x0 = [
                MBSim.parameters.q0;
                MBSim.parameters.qDot0;
                ];

            if obj.useMassMatrixForm
                if useMex
                    odeFun  = @(t,x) computeFirstOrderSystemRHS_mex(t, x, MBSim.system, MBSim.parameters);
                    massFun = @(t,x) computeFirstOrderMassMatrix_mex(t, x, MBSim.system);
                else
                    odeFun  = @(t,x) computeFirstOrderSystemRHS(t, x, MBSim.system, MBSim.parameters);
                    massFun = @(t,x) computeFirstOrderMassMatrix(t, x, MBSim.system);
                end
            else
                if useMex
                    odeFun = @(t,x) computeFirstOrderSystemRHS_MInv_mex(t, x, MBSim.system, MBSim.parameters);
                else
                    odeFun = @(t,x) computeFirstOrderSystemRHS_MInv(t, x, MBSim.system, MBSim.parameters);
                end
            end

            % Integration using the object-oriented approach
            if obj.showConsoleOutput
                fprintf("Starting integration (ode-direct, solver %s)...\n", ...
                    obj.odeObject.Solver);
            end

            obj.odeObject.ODEFcn = odeFun;
            obj.odeObject.InitialValue = x0;

            if obj.useMassMatrixForm
                obj.odeObject.MassMatrix = massFun;
            end
            tic;
            sol = solve(obj.odeObject, 0, MBSim.parameters.tEnd);
            tSim = toc;
            tout = sol.Time.';

            % Run simulation again with timing (if desired) for accuracy
            if obj.accurateTiming
                timingFun = @() solve(obj.odeObject, 0, MBSim.parameters.tEnd);
                tSim = timeit(timingFun, 1);
            end

            % ODE post-processing
            if obj.showConsoleOutput
                disp("Postprocessing integration results...")
            end

            if useMex
                simRes = getSimResFromStateTrajectory_mex(MBSim.system, tout, ...
                    sol.Solution(1:MBSim.system.nDoF,:), sol.Solution((MBSim.system.nDoF+1):end,:));
            else
                simRes = getSimResFromStateTrajectory(MBSim.system, tout, ...
                    sol.Solution(1:MBSim.system.nDoF,:), sol.Solution((MBSim.system.nDoF+1):end,:));
            end
            tFullODE = toc;

            simRes.computationTime = tSim;

            if obj.showConsoleOutput
                % Display meta data
                tStepODE = diff(tout);
                absTol = obj.odeObject.AbsoluteTolerance;
                relTol = obj.odeObject.RelativeTolerance;
                if obj.accurateTiming
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
