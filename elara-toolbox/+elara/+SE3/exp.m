function g = exp( xi )
    %% Exponential map for SE(3)
    % Implements the Exponential map for SE(3): exp : se(3) -> SE(3)
    %
    % Source: [Mül21, p.7], eq. 2.27
    % Follows the convention for se(3) elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Output g: Corresponding element of SE(3) in matrix form
    %
    % Requires Signal Processing Toolbox for the sinc function.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        xi (6,1)
    end

    om = xi(1:3);
    v  = xi(4:6);

    % General note: We explicitly include the formulas for the SO(3)
    % exponential map here so the auxiliary terms are only computed once

    % Rotation angle phi
    phi = norm(om);

    if phi
        omH = elara.SO3.skew(om);

        % Rotation axis n
        n = om / phi;

        % Compute auxiliary terms
        % Note: The arguments for sinc are divided by pi since we need the
        % *unnormalized* sinc function, but MATLAB sinc() is the normalized
        % version.
        alpha = sinc(phi/pi);
        beta  = sinc(phi/2/pi)^2;

        % Exponential map for SO(3)
        % [Mül21, p.5], eq. 2.6
        R = eye(3) + alpha*omH + 1/2 * beta * omH^2;

        % [Mül21, p.7], eq. 2.27
        x = ( eye(3) + beta/2 * omH + (1-alpha)*elara.SO3.skew(n)^2 ) * v;

    else
        R = eye(3);
        x = v;
    end

    g = elara.SE3.matrix(R, x);

end
