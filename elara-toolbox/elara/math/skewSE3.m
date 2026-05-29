function [X] = skewSE3(x)
    % SKEWSE3 Hat map for 6 dimensions, R6 -> se(3), i.e. 4x4 matrix
    %
    % The convention for the vector in R6 is
    %    x = [ omega; v ],
    % where omega is the angular and v the translational component.
    %
    % Implementation by:
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        x (6, 1)
    end
    % se(3) hat map, e.g. [Lee08, p. 38]
    X = [
        skewSO3(x(1:3)), x(4:end);
        0,0,0, 0
        ];
end