classdef MBExternalWrenchDefinition
    %% Class that defines the external wrenches in a multibody simulation
    % that act on the system
    %
    % Maximilian Herrmann
    % Leander Pfeiffer
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Start and end times of applied wrench / of the transient scaling
        % part
        startTime            (:,1)   double
        endTime              (:,1)   double

        % Type of interpolation:
        % 1: Constant force
        % 2: Linearly increasing "sawtooth"
        % 3: Linearly decreasing "sawtooth"
        % 4: Smooth impulse  (sinusoidal)
        % 5: Smooth increase (sinusoidal) to constant force
        % 6: Smooth decrease (sinusoidal) from constant force
        interpolationType    (:,1)   double

        % Maximum wrench applied; scaled based on type and current time
        % dimensions: 6,nFrames,nWrenchDefinitions
        wrench               (6,:,:) double
    end

    methods
        function obj = MBExternalWrenchDefinition()
            % No special constructor necessary
        end

        function obj = addWrench(obj, startTime, endTime, interpolationType, wrench)
            %% Add a wrench definition to the existing definitions
            arguments
                obj

                % Start time of applied wrench
                startTime            (1,1)   double
                % End time of applied wrench
                endTime              (1,1)   double
                % Type of interpolation (1-6)
                interpolationType    (1,1)   double
                % Maximum wrench applied. Scaled based on type and current time
                wrench               (6,:) double
            end
            obj.startTime(end+1)         = startTime;
            obj.endTime(end+1)           = endTime;
            obj.interpolationType(end+1) = interpolationType;
            if isempty(obj.wrench)
                obj.wrench = wrench;
            else
                obj.wrench = cat(3, obj.wrench, wrench);
            end
        end
    end
end