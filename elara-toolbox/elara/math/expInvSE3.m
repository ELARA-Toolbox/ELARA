function xi = expInvSE3( g )
    %% Inverse of the Exponential map for SE(3)
    % Implements the inverse exponential map for SE(3): 
    % expInv : SE(3) -> se(3)
    %
    % Source: [SCB14, p.471], eq. A.15
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
    
    omega = expInvSO3(R);
    v = expRTDInvSO3(omega) * p;

    xi = [ omega; v];

end
