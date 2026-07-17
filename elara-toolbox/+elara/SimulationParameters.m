classdef SimulationParameters
    % elara.SimulationParameters class containing all data for a specific simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Initial relative coordinates for the simulation
        % dimensions (1, nDoF)
        q0      (:,1)   double

        % Initial coordinate velocities for the simulation
        % dimensions (1, nDoF)
        qDot0   (:,1)   double

        % Simulation end time (= length of the simulation)
        tEnd    (1,1)   double

        % Gravity constant
        g       (1,1)   double  = 9.81;

        %% System Inputs u

        %%% Constant system inputs u
        % held constant throughout the simulation; if empty, it will be ignored.
        % dimensions (nInputs, 1)
        uConst          (:,1) double

        %%% Time-varying system inputs u
        % The time-varying input values at sample times uSampleTimes
        % will be interpolated at the simulation sample times.
        % If uSampleValues and/or uSampleTimes is empty, both will be
        % ignored.

        % Matrix of input values u with dimensions (nInputs, nSampleTimes)
        uSampleValues   (:,:) double

        % Vector of input sample times (corresponding to the values in
        % uSampleValues) with dimensions (nSampleTimes, 1)
        uSampleTimes    (:,1) double


        %% External wrenches acting on the frames
        % defined by elara.ExternalWrenchDefinition objects.
        % See elara.ExternalWrenchDefinition class for details.

        % Definition of body-fixed ext. wrenches
        externalWrench_b (1,1) elara.ExternalWrench

        % Definition of spatial ext. wrenches
        externalWrench_s (1,1) elara.ExternalWrench
    end
end
