function T = expRTDSE3( eta )
    %% Right-Trivialized Derivative of the Exponential map for SE(3)
    % Implements the right-trivialized derivative for the Exponential map
    % (also referred to as the right-trivialized tangent map) for SE(3): 
    % dexp : se(3) -> se(3) as the corresponding 6x6 linear matrix operator
    %
    % Source: [Mül21, p.8], Lemma 2.2
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

        % Rotation axis n
        n = om / phi;

        % Compute auxiliary terms
        % Note: The arguments for sinc are divided by pi since we need the
        % *unnormalized* sinc function, but MATLAB sinc() is the normalized
        % version.
        alpha = sinc(phi/pi);
        beta  = sinc(phi/2/pi)^2;
        delta = (1-alpha) / phi^2;

        omH = skewSO3(om);
        vH  = skewSO3(v);

        % Right-Trivialized Derivative for SO(3)
        % [Mül21, p.5], eq. 2.13
        dexp = eye(3) ...
            + beta / 2 * omH ...
            + (1 - alpha ) * skewSO3(n)^2;

        % [Mül21, p.8], Lemma 2.2
        Ddexp = ...
            + beta/2*vH ...
            + delta*(omH*vH + vH*omH) ...
            + (om.' * v) / (phi^2) ...
            * ( (alpha-beta) * omH + (beta/2 - 3*delta)* omH^2 );

        T = [
            dexp, zeros(3)
            Ddexp,   dexp
            ];
    else
        T = eye(6);
    end
end
