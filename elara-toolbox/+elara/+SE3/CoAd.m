function CoAd = CoAd(g)
    % Large co-adjoint representation (6x6 matrix) of an element of SE(3)
    % Follows the convention for se(3) elements in vector form: [omega; v]
    %
    % Input g: SE(3) element in (4x4) matrix representation
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

    % Co-adjoint representation, taken from [Lee08], eq. (3.123) / p. 84
    CoAd = [
        R.',       -R.' * elara.SO3.skew(p)  ;
        zeros(3),   R.'
        ];
end
