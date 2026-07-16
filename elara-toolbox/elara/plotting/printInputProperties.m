function tInputs = printInputProperties(MBSys)
    %% Display input properties of a MB system as a table

    % Disable warning appearing when we convert an object to struct
    warning('off','MATLAB:structOnObject');

    %% Get input information

    % Max. nr. of inputs one frame can have
    maxNrInputs = max(MBSys.frames.uIndices(2,:)-MBSys.frames.uIndices(1,:))+1;

    % Matrix with the indices of all inputs of each frame (padded with
    % zeros)
    frameInputMatrix = zeros(MBSys.nFrames,maxNrInputs);
    for iFrm = 1:MBSys.nFrames
        frameInputs = MBSys.frames.uIndices(1,iFrm):MBSys.frames.uIndices(2,iFrm);
        frameInputMatrix(iFrm,1:length(frameInputs)) = frameInputs;
    end

    % Find link and type of all inputs

    inputType = strings(MBSys.nInputs,1);
    inputLink = zeros(MBSys.nInputs,1);

    for iInput = 1:MBSys.nInputs

        % Get indices of all frames, on which the input acts
        [uFrms,~] = find(frameInputMatrix == iInput);

        % Get link, on which the input acts (should be only 1)
        uLink = unique(MBSys.frames.linkIndex(uFrms));
        assert(isscalar(uLink), "Input acts on more than one link.");
        inputLink(iInput) = uLink;

        % Get joint types of frames, on which the input acts (should be
        % only 1 joint type)
        uType = unique(MBSys.frames.jointType(uFrms));
        assert(isscalar(uLink), "Input affects multiple joint types.");

        switch uType
            case 1
                inputType(iInput) = "Joint Actuation";

            case 2
                inputType(iInput) = "Cable Actuation";

            otherwise
                error("Joint type not defined.")
        end
    end

    %% Display as table
    tInputs = table((1:MBSys.nInputs).', inputType, inputLink, ...
        'VariableNames', ["Input Nr.", "Type", "Link"]);
    disp("System Input Properties:")
    disp(tInputs);
end