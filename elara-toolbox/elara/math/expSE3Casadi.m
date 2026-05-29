function g = expSE3Casadi( xi )
    %% Approximation of the exponential map for SE(3)
    % Implements the an approximation of the Lie group exponential map 
    % for SE(3): exp : se(3) -> SE(3).
    %
    % It is computed directly via the series expansion in [Mül21, p.4], 
    % eq. 2.1, which avoids the discontinuity of the closed formula.
    %
    % Convention for se3 elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Output g: Corresponding element of SE3 in matrix form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        xi (6,1)
    end

    xiC = casadi.SX.sym('xi', 6, 1);
    x = xiC(1:3);
    y  = xiC(4:6);
    % Rotation angle phi
    phiC = norm(x);

    % Exponential map for SO3
    % [Mül21, p.5], eq. 2.6
    R = eye(3) + (sin(phiC)/phiC)*skewSO3(x) + (1-cos(phiC)/phiC^2) * skewSO3(x)^2;
    h = x.'*y / (phiC)^2;
    x = 1/(phiC)^2*(eye(3)-R)*skewSO3(x)*y +h*y;

    fun_zero = casadi.Function('FR0', {x, y}, {[eye(3), x; 0,0,0,1]});
    fun_nonzero = casadi.Function('FRNZ', {x, y}, {[R, x; 0,0,0,1]});

    om = xi(1:3);
    v  = xi(4:6);
    phi = norm(om);   
    g = if_else(phi>0, fun_nonzero(om,v), fun_zero(om,v));
end
