classdef ExternalWrench
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
        maximumWrench               (6,:,:) double
    end

    methods
        function obj = addWrench(obj, startTime, endTime, interpolationType, maximumWrench)
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
                maximumWrench               (6,:) double
            end
            obj.startTime(end+1)         = startTime;
            obj.endTime(end+1)           = endTime;
            obj.interpolationType(end+1) = interpolationType;
            if isempty(obj.maximumWrench)
                obj.maximumWrench = maximumWrench;
            else
                obj.maximumWrench = cat(3, obj.maximumWrench, maximumWrench);
            end
        end
        function f_frame = getCurrentWrench(externalWrench, nFrames, t)
            %% Compute external frame wrenches for current time
            %
            % Maximilian Herrmann
            % Leander Pfeiffer
            % Chair of Automatic Control
            % TUM School of Engineering and Design
            % Technical University of Munich
            arguments (Input)
                % Object defining the external wrenches
                externalWrench (1,1) elara.ExternalWrench

                % Number of frames
                nFrames             (1,1) double

                % Current simulation time
                t                   (1,1) double
            end
            arguments (Output)
                % Array of external frame forces (wrenches)
                % with dimensions (6,nFrames)
                f_frame             (6,:) double
            end

            nDefinitions = length(externalWrench.startTime);

            assert(...
                (length(externalWrench.endTime) == nDefinitions) && ...
                (length(externalWrench.interpolationType) == nDefinitions) && ...
                (size(externalWrench.maximumWrench,3) == nDefinitions), ...
                'Dimensions for wrench definition doe not match.');

            f_frame = zeros(6,nFrames);

            for iDef = 1:nDefinitions
                t_start = externalWrench.startTime(iDef);
                t_end = externalWrench.endTime(iDef);
                interpType = externalWrench.interpolationType(iDef);

                if t_start > t && interpType ~= 6
                    continue
                elseif t_end <= t && interpType ~= 5
                    continue
                end

                switch interpType
                    case 1 % Constant
                        scalingFactor = 1;
                    case 2 % Rising
                        scalingFactor = (t - t_start) / (t_end - t_start);
                    case 3 % Falling
                        scalingFactor = (t_end - t) / (t_end - t_start);
                    case 4 % Smooth impulse
                        scalingFactor = (1 - cos(2*pi*(t - t_start)/ (t_end - t_start)))/2;
                    case 5 % Smooth increase
                        if t_end <= t
                            scalingFactor = 1;
                        else
                            scalingFactor = (1 - cos(pi*(t - t_start)/(t_end - t_start)))/2;
                        end
                    case 6 % Smooth decrease
                        if t_start > t
                            scalingFactor = 1;
                        else
                            scalingFactor = (1 + cos(pi*(t - t_start) / (t_end - t_start)))/2;
                        end
                    otherwise
                        scalingFactor = 0;
                end

                f_frame = f_frame + scalingFactor * externalWrench.maximumWrench(:,:,iDef);
            end
        end
    end
end
