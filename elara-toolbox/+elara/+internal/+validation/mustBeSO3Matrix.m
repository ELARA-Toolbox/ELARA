function mustBeSO3Matrix(R)
    %% Validate that the input is an array of SO(3) matrices
    % where the array has dimensions (3,3,:,:...,:).
    % The first two dimensions are the rows and columns of the matrices;
    % the number of remaining dimensions is arbitrary.
    assert(size(R,1) == 3 && size(R,2) == 3, ...
        "Input is not an SO(3) matrix: Matrices must be 3x3.");

    RArray = reshape(R, 3, 3, []);
    for iMat = 1:size(RArray,3)
        mustBeSO3MatrixSingle(RArray(:,:,iMat));
    end
end
function mustBeSO3MatrixSingle(R)
    %% Validate that the input is an SO(3) matrix

    % Check if matrix is real
    assert( isreal(R), ...
        "Input is not an SO(3) matrix: Matrix must be real." ...
        );

    % Check size
    assert( size(R,1) == 3 && size(R,2) == 3, ...
        "Input is not an SO(3) matrix: Matrix must be 3x3." ...
        );

    % Check determinant
    assert( abs(det(R) - 1 ) < 100*eps, ...
        "Input is not an SO(3) matrix: Matrix must have determinant 1." ...
        );

    % Check orthogonality
    assert( all( abs((R.'*R) - eye(3)) < 100*eps, "all"), ...
        "Input is not an SO(3) matrix: Matrix must be orthogonal." ...
        );
end
