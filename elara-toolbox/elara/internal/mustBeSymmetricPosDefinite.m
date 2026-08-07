function mustBeSymmetricPosDefinite(A)
    %% Validate that the input is an array of positive semi-definite matrices
    dim = size(A,1);
    A = reshape(A, dim, dim, []);

    for iMat = 1:size(A,3)
        mustBeSymmetricPosDefiniteSingle(A(:,:,iMat));
    end
end
function mustBeSymmetricPosDefiniteSingle(A)
    %% Validate that the input is a positive semi-definite matrix

    % Check if matrix is positive definite
    assert( all(eig(A) > 10*eps), ...
        "Matrix is not positive definite.");

    % Check if matrix is symmetric
    assert( all( A - A.' < 10*eps, "all" ), ...
        "Matrix is not symmetric.");
end