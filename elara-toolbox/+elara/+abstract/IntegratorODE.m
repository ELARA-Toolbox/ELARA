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
end
