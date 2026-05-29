function xi = cayInvSE3( g )
    %% Inverse of the Cayley map for SE(3)
    % Implements the inverse Cayley map for SE(3): cay : SE(3) -> se(3)
    %
    % Source: [Dem+14, p.10]
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input g:   Element of SE3 in matrix form
    % Output xi: Corresponding se(3) element in *vector* form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        g (4,4)
    end

    R = g(1:3, 1:3);
    p = g(1:3, 4);

    omega = cayInvSO3( R );
    v     = 2 * ( (R + eye(3)) \ p );

    xi = [ omega; v];
end

