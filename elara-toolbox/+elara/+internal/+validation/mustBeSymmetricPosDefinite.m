function mustBeSymmetricPosDefinite(A)
    %% Validate that the input is an array of positive-definite matrices
    dim = size(A,1);
    assert(size(A,2) == dim, "Matrix must be square.");
    AArray = reshape(A, dim, dim, []);

    for iMat = 1:size(AArray,3)
        mustBeSymmetricPosDefiniteSingle(AArray(:,:,iMat));
    end
end
function mustBeSymmetricPosDefiniteSingle(A)
    %% Validate that the input is a positive-definite matrix

    % Check if matrix is symmetric
    assert( all(abs(A - A.') < 10*eps, "all"), ...
        "Matrix is not symmetric.");

    % Check if matrix is positive definite
    assert( all(eig(A) > 10*eps), ...
        "Matrix is not positive definite.");
end
