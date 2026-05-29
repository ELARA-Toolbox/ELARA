function [X] = skew(x)
    % SKEW Hat map for 2, 3 and 6 dimensions:
    % R1 -> so(2), i.e. 2x2 matrix
    % R3 -> so(3), i.e. 3x3 matrix
    % R6 -> se(3), i.e. 4x4 matrix
    %
    % Important:
    % For se3, the convention for the vector in R6 is
    %    x = [ omega; v ],
    % where omega is the angular and v the translationalcomponent.
    %
    % Implementation by:
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        x (:, 1)
    end

    if isscalar(x)
        % so(2) hat map
        % e.g. [Lee08, p. 26]
        X = [ 0, -x; x, 0 ];
    elseif length(x) == 3
        % so(3) hat map
        % e.g. [Lee08, p. 28]
        X = skewSO3(x);
    elseif length(x) == 6
        % se(3) hat map
        % e.g. [Lee08, p. 38]
        X = [
            skewSO3(x(1:3)), x(4:end);
            zeros(1, 3),     0
            ];
    else
        error('Input vector has wrong dimension. Valid nr. of elements: 1, 3 or 6.')
    end
end