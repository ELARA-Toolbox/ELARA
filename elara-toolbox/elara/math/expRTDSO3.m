function T = expRTDSO3( om )
    %% Right-Trivialized Derivative of the Exponential map for SO(3)
    % Implements the right-trivialized derivative for the Exponential map
    % (also referred to as the right-trivialized tangent map) for SO(3):
    % dexp : so(3) -> so(3) as the corresponding 6x6 linear matrix operator
    %
    % Source: [Mül21, p.5], eq. 2.13
    %
    % Input om:   so(3) element in *vector* form (omega)
    % Output Lambda: Corresponding rotation matrix
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        om (3,1)
    end

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

        omH = skewSO3(om);

        T = eye(3) ...
            + beta / 2 * omH ...
            + (1 - alpha ) * skewSO3(n)^2;
    else
        T = eye(6);
    end
end
