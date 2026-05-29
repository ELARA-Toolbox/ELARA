function f = getCayRTDSE3dtFunction(q)
    %% Get function handles for cayRTDSE3dt Function
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Variable that defines the type of the output function:
        % Numeric or Casadi function handles (if q is numeric, output
        % functions are numeric as well, etc.)
        q (:,:) = casadi.MX.sym('q',1,1);
    end

    % Use persistent functions for casadi functions to make sure there are
    % no duplicate function definitions in the graph
    persistent fun

    if isa(q, "double")
        f = @cayRTDSE3dt_sep;
    else
        if isempty(fun)

            omS     = casadi.MX.sym('om', 3, 1);
            omS2    = casadi.MX.sym('om2', 3, 1);
            vS      = casadi.MX.sym('v', 3, 1);
            vS2     = casadi.MX.sym('v2', 3, 1);

            T = cayRTDSE3dt_sep(omS, vS, omS2, vS2);

            fun = casadi.Function('cayRTDSE3dt', ...
                {omS, vS, omS2, vS2}, {T});
        end
        f = fun;
    end

    function RTD_dt = cayRTDSE3dt_sep(om, v, om_dot, v_dot)
        % Wrapper for cayRTDSE3dt for separate omega, v inputs (instead of
        % xi)
        RTD_dt = cayRTDSE3dt([om; v], [om_dot; v_dot]);
    end
end