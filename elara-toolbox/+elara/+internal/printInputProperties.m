function tInputs = printInputProperties(system)
    %% Display input properties of an ELARA system as a table

    % Disable warning appearing when we convert an object to struct;
    % Restore the caller's warning state after constructing the display tables.
    warningState = warning("query", "MATLAB:structOnObject");
    warningCleanup = onCleanup(@() warning(warningState));
    warning("off", "MATLAB:structOnObject");

    %% Get input information

    % Max. nr. of inputs one frame can have
    maxNrInputs = max(system.frames.uIndices(2,:)-system.frames.uIndices(1,:))+1;

    % Matrix with the indices of all inputs of each frame (padded with
    % zeros)
    frameInputMatrix = zeros(system.nFrames,maxNrInputs);
    for iFrm = 1:system.nFrames
        frameInputs = system.frames.uIndices(1,iFrm):system.frames.uIndices(2,iFrm);
        frameInputMatrix(iFrm,1:length(frameInputs)) = frameInputs;
    end

    % Find link and type of all inputs

    inputType = strings(system.nInputs,1);
    inputLink = zeros(system.nInputs,1);

    for iInput = 1:system.nInputs

        % Get indices of all frames, on which the input acts
        [uFrms,~] = find(frameInputMatrix == iInput);

        % Get link, on which the input acts (should be only 1)
        uLink = unique(system.frames.linkIndex(uFrms));
        assert(isscalar(uLink), "Input acts on more than one link.");
        inputLink(iInput) = uLink;

        % Get joint types of frames, on which the input acts (should be
        % only 1 joint type)
        uType = unique(system.frames.jointType(uFrms));
        assert(isscalar(uLink), "Input affects multiple joint types.");

        switch uType
            case 1
                inputType(iInput) = "Joint Actuation";

            case 2
                inputType(iInput) = "Tendon Actuation";

            otherwise
                error("Joint type not defined.")
        end
    end

    %% Display as table
    tInputs = table((1:system.nInputs).', inputType, inputLink, ...
        'VariableNames', ["Input Nr.", "Type", "Link"]);
    disp("System Input Properties:")
    disp(tInputs);
end