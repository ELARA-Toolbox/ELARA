function [x] = skewInv(X)
    % SKEWINV Inverse hat map for 2, 3 and 6 dimensions:
    % so(2) -> R1
    % so(3) -> R3
    % se(3) -> R6
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
        X (:, :)
    end

    if all(size(X) == 2)
        % so(2) hat map
        % e.g. [Lee08, p. 26]
        x = X(2, 1);
    elseif all(size(X) == 3)
        % so(3) hat map
        % e.g. [Lee08, p. 28]
        x = skewInv_so3(X);
    elseif all(size(X) == 4)
        % se(3) hat map
        % e.g. [Lee08, p. 38]
        x = [
            skewInv_so3(X(1:3, 1:3));
            X(1:3, 4);
            ];
    else
        error('Input matrix has wrong dimension. Valid dimensions: 2x2, 3x3 or 4x4.')
    end
end

% Inverse hat map for so(3)
function x = skewInv_so3(X)
    x = [ X(3,2); X(1,3); X(2,1) ];
end

