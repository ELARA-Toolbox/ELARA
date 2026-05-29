function f = getSE3Functions(q)
    %% Get function handles for the SE3 Functions
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Variable that defines the type of the output functions:
        % Numeric or Casadi function handles (if q is numeric, output
        % functions are numeric as well, etc.)
        q (:,:) = casadi.MX.sym('q',1,1);
    end

    % Use persistent functions for casadi functions to make sure there are
    % no duplicate function definitions in the graph
    persistent fun

    if isa(q, "double")
        f = getSE3FunctionsNumeric;
    else
        if isempty(fun)
            fun = getSE3FunctionsCasadi;
        end
        f = fun;
    end