function g = expSE3Approx( xi )
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
    xiH = skewSE3(xi);
    g = zeros(4,4);
    for iTerm = 1:25
        g = g + xiH^(iTerm-1) / factorial(iTerm-1);
    end
end
