function Lambda = expSO3( om )
    %% Exponential map for SO(3)
    % Implements the exponential map for SO(3): exp : so(3) -> SO(3)
    %
    % Source: [Mül21, p.5], eq. 2.6
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

    omH = skewSO3(om);

    % Compute auxiliary terms
    % Note: The arguments for sinc are divided by pi since we need the
    % *unnormalized* sinc function, but MATLAB sinc() is the normalized
    % version.
    alpha = sinc(phi/pi);
    beta  = sinc(phi/2/pi)^2;

    Lambda = eye(3) + alpha*omH + 1/2 * beta * omH^2;
end
