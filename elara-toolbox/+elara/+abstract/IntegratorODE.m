classdef (Abstract) IntegratorODE < elara.abstract.Integrator
    %% Abstract class defining an ODE integrator for a ELARA Simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % If true, solve the ODE in the mass matrix form
        %    M(x) x-dot = f(x);
        % the mass matrix function is passed to the ode solver.
        % If false, solve the ODE in the form
        %    x-dot = f(x),
        % where the mass matrix is included (as inverse in f(x). No
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
