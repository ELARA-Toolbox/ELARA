function T = expRTDInvSE3( eta )
    %% Inverse Right-Trivialized Derivative of the Exponential map for SE(3)
    % Implements the inverse of the right-trivialized derivative for the 
    % exponential map (also referred to as the right-trivialized tangent 
    % map) for SE(3): 
    % dexp : se(3) -> se(3) as the corresponding 6x6 linear matrix operator
    %
    % Source: [Mül21, p.9], Lemma 2.3
    %
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input eta: se(3) element in *vector* form
    % Output T: 6x6 matrix
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        eta (6,1)
    end


    om = eta(1:3);
    v  = eta(4:end);

    % General note: We explicitly include the formulas for the SO3
    % exponential map here so the auxiliary terms are only computed once

    % Rotation angle phi
    phi = norm(om);

    % Check if we have non-zero rotation
    if phi

        % Compute auxiliary terms
        % Note: The arguments for sinc are divided by pi since we need the
        % *unnormalized* sinc function, but MATLAB sinc() is the normalized
        % version.
        beta  = sinc(phi/2/pi)^2;
        gamma = cos(phi/2) / sinc(phi/2/pi);

        omH = skewSO3(om);
        vH  = skewSO3(v);

        % Inverse Right-Trivialized Derivative for SO(3)
        % [Mül21, p.5], eq. 2.16
        dexpInv = eye(3) ...
            - 1/2 * omH ...
            + 1/(phi^2) * (1-gamma) * omH^2;

        % [Mül21, p.9], Lemma 2.3
        DdexpInv = ...
            - 1/2 * vH ...
            + 1/(phi^2)* (1-gamma)*(omH*vH + vH*omH) ...
            + (om.' * v) / (phi^4) * ( 1/beta + gamma - 2 )* omH^2 ;

        T = [
            dexpInv, zeros(3)
            DdexpInv,   dexpInv
            ];
    else
        T = [eye(3), zeros(3); -0.5*skewSO3(v), eye(3)];
    end
end
