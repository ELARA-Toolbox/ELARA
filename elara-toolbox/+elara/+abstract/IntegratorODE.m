classdef (Abstract) IntegratorODE < elara.abstract.Integrator
    %% Abstract class defining an ODE integrator for an ELARA simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % If true, solve the ODE in the mass matrix form
        %    M(x) x-dot = f(x);
        % the mass-matrix function is passed to the ODE solver.
        % If false, solve the ODE in the form
        %    x-dot = f(x),
        % where f(x) includes the inverse mass matrix. No
        % additional options are passed to the solver.
        useMassMatrixForm (1,1) logical = false;
    end

    methods
        function fhs = plotSolverStats(~, simulation)
            arguments
                ~
                simulation (1,1) elara.Simulation
            end
            fhs = elara.plot.solverStatsODE(simulation.results, ...
                "nameString", simulation.Name);
        end
    end

    methods (Static, Access = protected)
        function validateSystemInputs(system, simPars)
            % Check fixed input dimensions before running an ODE integration.
            arguments
                system  (1,1) elara.SystemNum
                simPars (1,1) elara.SimulationParameters
            end

            assert(isempty(simPars.uConst) || numel(simPars.uConst) == system.nInputs, ...
                'elara:InvalidConstantInputDimensions', ...
                'The constant-input vector must contain nInputs elements.');

            % If either of the sampled-input arrays is empty,
            % dynamic/sampled inputs are ignored in the ODE functions.
            if ~isempty(simPars.uSampleValues) && ~isempty(simPars.uSampleTimes)
                assert(size(simPars.uSampleValues,1) == system.nInputs, ...
                    'elara:InvalidSampledInputDimensions', ...
                    'The time-varying input matrix must have one row per system input.');
                assert(size(simPars.uSampleValues,2) == numel(simPars.uSampleTimes), ...
                    'elara:InvalidInputSampleCount', ...
                    'The number of sample times must match the number of samples in uSampleValues.');
            end
        end
    end
end
