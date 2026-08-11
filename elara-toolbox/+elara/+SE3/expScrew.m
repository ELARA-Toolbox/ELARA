function g = expScrew( X, theta )
    %% Exponential map for SE(3) with constant screw axis
    % Implements the Exponential map for SE(3): exp : se(3) -> SE(3)
    % with a constant screw axis (twist) X and magnitude ("angle") theta.
    %
    % We use the standard formulas for the SE(3) exponential map, e.g.,
    % from [MLS94].
    %
    % Important: This function only implements the case where the
    % rotational component is nonzero (i.e., omega ~= 0), which corresponds
    % to pure rotations or screw motions (rotation and translation). The
    % case of pure translation (with omega = 0) is not implemented.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Screw axis / twist
        X       (6,1)
        % Magnetude / angle
        theta   (1,1)
    end
    om = X(1:3);
    v  = X(4:6);
    omh = elara.SO3.skew(om);

    % Formula (2.14), p. 28 in [MLS94]
    R = eye(3) + omh*sin(theta) + omh^2*(1-cos(theta));

    % Formula (2.36), p. 42 in [MLS94]
    x = (eye(3) - R)*omh*v + om*om.'*v*theta;

    g = [R, x; 0, 0, 0, 1];
end
