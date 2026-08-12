function tf = isMexAvailable(functionName)
    %ISMEXAVAILABLE Determine whether a qualified name resolves to a MEX file.
    % MATLAB's EXIST function does not report packaged MEX functions, so use
    % WHICH and verify the platform-specific extension instead.
    arguments
        functionName (1,1) string
    end

    functionPath = string(which(functionName));
    tf = strlength(functionPath) > 0 && endsWith(functionPath, "." + mexext);
end
