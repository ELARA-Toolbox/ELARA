function f_frame = getExternalStepWrenches(extWrenchDefinition, nFrames, t)
    %% Compute external frame wrenches for current time
    %
    % Maximilian Herrmann
    % Leander Pfeiffer
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        % Object defining the external wrenches
        extWrenchDefinition (1,1) MBExternalWrenchDefinition

        % Nr. of frames
        nFrames             (1,1) double

        % Current simulation time
        t                   (1,1) double
    end
    arguments (Output)
        % Array of external frame forces (wrenches)
        % with dimensions (6,nFrames)
        f_frame             (6,:) double
    end

    nDefinitions = length(extWrenchDefinition.startTime);

    assert(...
        (length(extWrenchDefinition.endTime) == nDefinitions) && ...
        (length(extWrenchDefinition.interpolationType) == nDefinitions) && ...
        (size(extWrenchDefinition.wrench,3) == nDefinitions), ...
        'Dimensions for wrench definition doe not match.');

    f_frame = zeros(6,nFrames);

    for iDef = 1:nDefinitions
        t_start = extWrenchDefinition.startTime(iDef);
        t_end = extWrenchDefinition.endTime(iDef);
        interpType = extWrenchDefinition.interpolationType(iDef);

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

        f_frame = f_frame + scalingFactor * extWrenchDefinition.wrench(:,:,iDef);
    end
end