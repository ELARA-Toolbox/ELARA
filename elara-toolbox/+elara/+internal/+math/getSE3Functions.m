function f = getSE3Functions(q)
    %% Get function handles for the SE(3) functions
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Variable that defines the type of the output functions:
        % Numeric or CasADi function handles (if q is numeric, output
        % functions are numeric as well, etc.)
        q (:,:) = casadi.MX.sym('q',1,1);
    end

    % Use persistent functions for CasADi functions to make sure there are
    % no duplicate function definitions in the graph
    persistent fun

    if isa(q, "double")
        f = elara.internal.math.getSE3FunctionsNumeric;
    else
        if isempty(fun)
            fun = elara.internal.math.getSE3FunctionsCasadi;
        end
        f = fun;
    end
