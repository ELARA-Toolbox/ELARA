function tLinks = printLinkProperties(links)
    %% Display link properties of an ELARA system as a table

    % Disable warning appearing when we convert an object to struct;
    % Restore the caller's warning state after constructing the display tables.
    warningState = warning("query", "MATLAB:structOnObject");
    warningCleanup = onCleanup(@() warning(warningState));
    warning("off", "MATLAB:structOnObject");

    %% Print link properties

    linkClasses = ["Rigid", "Flexible"];
    linkClassIndices = {find([links.isRigid]), find(~[links.isRigid])};


    for iClass = 1:numel(linkClasses)
        tLinks = table();
        for iLink = linkClassIndices{iClass}
            linkStr = struct(links(iLink));
            linkStr = prepareStructFields(linkStr);
            tLinks = [tLinks;struct2table(linkStr, "AsArray", 1)];
        end
        tIndex = table(linkClassIndices{iClass}.');
        tIndex.Properties.VariableNames = "Nr.";
        tLinks = [tIndex, tLinks];

        disp("System Link Properties (" + linkClasses(iClass) + " Links):");
        disp(tLinks);
    end
end

function s = prepareStructFields(s)
    % Prepare struct for display output:
    %  * Check a struct for fields that are column vectors and transpose
    %    them to obtain row vectors
    %  * Remove fields for SE3 transformations, i.e., fields starting with
    %    "g_"
    fieldNames = fields(s);

    for iFld = 1:length(fieldNames)

        % Transpose column vectors
        if size(s.(fieldNames{iFld}),2) == 1
            s.(fieldNames{iFld}) = (s.(fieldNames{iFld})).';
        end
        % Put non-scalar and non-twist variables into cells
        if ~(isscalar(s.(fieldNames{iFld})) || all(size(s.(fieldNames{iFld}), [1,2]) ==[1,6]))
            s.(fieldNames{iFld}) = {(s.(fieldNames{iFld}))};
        end
        % Remove transformations
        if contains(fieldNames{iFld}, "g_")
            s = rmfield(s, fieldNames{iFld});
        end
    end
end