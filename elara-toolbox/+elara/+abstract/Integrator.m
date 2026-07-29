classdef (Abstract) Integrator
    %% Abstract class defining an integrator for a ELARA Simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Whether or not to print simulation meta data to the
        % console
        showConsoleOutput (1,1) logical = true;

        % if true, time is measured using an simulation run that is
        % measured using the timeit() function, which results in
        % much higher overall runtime (but is more accurate, of
        % course)
        accurateTiming (1,1) logical = false;
    end
    properties(Abstract,Constant)
        % Defines the integrator type; internally used for post-processing
        % etc.
        type           (1,1) string {mustBeMember(type, ["varint", "ode"])}
    end

    methods (Abstract)
        % Main integration function; must have arguments and output
        %   simRes = simulateSystem(obj, MBSim)
        % with classes:
        %   simRes: elara.SimulationResults
        %   MBSim: MB-Simulation
        simulateSystem
    end
end
