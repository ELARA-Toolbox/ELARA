function omega = cayInvSO3( Lambda )
    %% Inverse of the Cayley map for SO(3)
    % Implements the inverse Cayley map for SO(3): cayInv : SO3(3) -> so(3)
    %
    % Source: [Dem+14, p.9], eq. 15
    %
    % Input Lambda: Rotation matrix
    % Output omega: Corresponding so(3) element in *vector* form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        Lambda (3,3)
    end

    omegaH = 2 / (1 + trace(Lambda) ) * (Lambda - Lambda.');

    omega = skewInv(omegaH);
end

