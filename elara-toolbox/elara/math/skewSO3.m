function [X] = skewSO3(x)
    % SKEWSO3 Hat map for so(3) / 3 dimensions R3 -> so(3), i.e. 3x3 matrix
    %
    % Implementation by:
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        x (3, 1)
    end

    X = [
        0,    -x(3),  x(2)
        x(3),     0, -x(1)
        -x(2), x(1),     0
        ];
end