function mustBeSymmetricPosDefinite(A)
    %% Validate that the input is a positive semi-definite matrix

    % Check if matrix is positive definite
    assert( all(eig(A) > 10*eps), ...
        "Matrix is not positive definite.");

    % Check if matrix is symmetric
    assert( all( A - A.' < 10*eps, "all" ), ...
        "Matrix is not symmetric.");
end