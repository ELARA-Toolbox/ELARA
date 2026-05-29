function omega = expInvSO3( R )
    %% Inverse of the Exponential map for SO(3)
    % Implements the inverse exponential map for SO(3):
    % expInv : SO(3) -> so(3)
    %
    % Source: [SCB14, p.470], eq. A.8
    %
    % Input Lambda: Rotation matrix
    % Output omega: Corresponding so(3) element in *vector* form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        R (3,3)
    end

    % Check if we have zero rotation
    if (trace(R) - 3) == 0
        omega = zeros(3,1);
    else

        theta = acos(1/2 * (trace(R) - 1));
        % Todo: Check if |theta| < pi?

        omegaH = theta/2/sin(theta) * (R - R.');
        omega  = skewInv(omegaH);

    end
end
