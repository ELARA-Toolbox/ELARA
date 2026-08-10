function CoAd = CoAdSeparate(R,p)
    %% Large co-adjoint representation (6x6 matrix) of an element of SE(3)
    % With separate R and p arguments
    % Follows the convention for se(3) elements in vector form: [omega; v]
    %
    % Inputs: See below
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        R (3,3) % SO(3) rotation matrix
        p (3,1) % R^3 position vector
    end

    % Co-adjoint representation, taken from [Lee08], eq. (3.123) / p. 84
    CoAd = [
        R.',       -R.' * elara.SO3.skew(p)  ;
        zeros(3),   R.'
        ];
end
