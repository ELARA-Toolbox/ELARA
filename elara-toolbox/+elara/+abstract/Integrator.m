classdef (Abstract) Integrator
    %% Abstract class defining an integrator for an ELARA simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Whether to print simulation metadata to the console
        showConsoleOutput (1,1) logical = true;

        % If true, benchmark integration with timeit. This improves timing
        % accuracy but increases the overall runtime.
        accurateTiming (1,1) logical = false;
    end
    properties(Abstract,Constant)
        % Defines the integrator type; internally used for post-processing
        % etc.
        type           (1,1) string {mustBeMember(type, ["varint", "ode"])}
    end

    methods (Abstract)
        % Main integration function; must have arguments and output
        %   simRes = simulateSystem(obj, simulation)
        % with classes:
        %   simRes: elara.SimulationResults
        %   simulation: elara.Simulation
        simulateSystem

        % Function to plot the solver metadata/statistics from a completed
        % simulation. Must have the signature:
        %  figureHandles = plotSolverStats(obj, simulation)
        plotSolverStats
    end
end
