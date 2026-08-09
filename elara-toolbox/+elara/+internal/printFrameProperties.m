function tFrames = printFrameProperties(system)
    %% Display frame properties of an ELARA system as a table

    % Disable warning appearing when we convert an object to struct;
    % Restore the caller's warning state after constructing the display tables.
    warningState = warning("query", "MATLAB:structOnObject");
    warningCleanup = onCleanup(@() warning(warningState));
    warning("off", "MATLAB:structOnObject");

    % Get frames as a suitable struct
    frameStr = struct(system.frames);
    frameStr.ancestors = frameStr.ancestors.';
    frameStr = prepareStruct(frameStr, system.nFrames);

    % Construct table
    tIndex = table((1:system.nFrames).');
    tIndex.Properties.VariableNames = "Nr.";
    tFrames = [tIndex, struct2table(frameStr)];

    disp("System Frame Properties:")
    disp(tFrames);
end

function s = prepareStruct(s, nFrames)
    % Prepare the struct for display output:
    % * Check a struct for fields that are row vectors and transpose
    %   them to obtain column vectors
    % * Remove fields that are empty or have wrong dimensions to display
    %   them in a table
    fieldNames = fields(s);

    for iFld = 1:length(fieldNames)
        % Transpose row vectors
        if size(s.(fieldNames{iFld}),1) == 1
            s.(fieldNames{iFld}) = s.(fieldNames{iFld}).';
        end

        % Transpose matrices with wrong dimensions
        if size(s.(fieldNames{iFld}),2) == nFrames && size(s.(fieldNames{iFld}),3) == 1
            s.(fieldNames{iFld}) = s.(fieldNames{iFld}).';
        end
        
        % Remove fields
        if size(s.(fieldNames{iFld}),1) ~= nFrames || isempty(s.(fieldNames{iFld}))
            s = rmfield(s, fieldNames{iFld});
        end        
    end
end