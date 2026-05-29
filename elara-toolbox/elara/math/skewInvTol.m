    function [x] = skewInvTol(X, options)
    % SKEWINV Inverse hat map for 2 and 3 dimensions which also checks the
    % tolerance of skew-symmetry of the input matrix (if it is numeric).
    % R^(n x n) -> R^n
    %
    % The function displays a warning if the argument deviates too much
    % from the ideal skew-symmetric matrix, i.e. if X + X' ~= 0.
    % If the deviation is too large, the function throws an error.
    %
    % Optional Name-Value pair arguments:
    % - Tolerance: Allowed deviation from the skew-symmetric form until an
    %   error is thrown, calculated by norm(X + X').
    %   Set to a e.g. 1 to disable the error check.
    % - NoWarnings: Suppresses the warning about numeric deviation from the
    %   ideal skew-symmetric form
    %
    % Implementation by:
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        X (:, :)
        options.Tolerance    (1,1) {mustBeNumeric} = 1e-17
        options.NoWarnings   (1,1) logical = 0
    end

    % For numeric matrices:
    % Check if the argument is a proper skew-symmetrical matrix
    % M + M' = 0 must hold
    if isnumeric(X)


        if ~all(X + X' == 0)

            residual = norm( X + X' );

            %if isnumeric(residual)
            if double(residual) > options.Tolerance
                % Throw error if deviation exceeds defined tolerance
                error([
                    'Argument is not a skew-symmetrical matrix. ', ...
                    'Specified tolerance threshold exceeded. ', ...
                    'Residual norm: ', num2str(residual)]);
            else
                % Else, display at least a warning for the deviation (if
                % warning is not disabled)
                if ~options.NoWarnings
                    warning([
                        'Numeric deviation from skew-symmetric form. ', ...
                        'Residual norm: ', num2str(double(residual))]);
                end
            end
        end
    end

    % Extract elements from the matrix
    if all(size(X) == 2)
        % 2D hat map
        % e.g. [Lee08, p. 26]
        x = X(2, 1);
    elseif all(size(X) == 3)
        % 3D hat map
        % e.g. [Lee08, p. 28]
        x = [ X(3,2); X(1,3); X(2,1) ];
    else
        error('Not implemented for this dimension.')
    end
end
