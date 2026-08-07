function mustBeSE3Matrix(g)
    %% Validate that the input is a array of SE3 matrices
    % where the array has dimensions (4,4,:,:...,:).
    % The first two dimensions are the rows and columns of the matrices;
    % the number of remaining dimensions is arbitrary.
    assert(size(g,1) == 4 && size(g,2) == 4, ...
        "Input is not an SE3 matrix: Matrices must be 4x4.");

    gArray = reshape(g, 4, 4, []);
    for iMat = 1:size(gArray,3)
        mustBeSE3MatrixSingle(gArray(:,:,iMat));
    end
end
function mustBeSE3MatrixSingle(g)
    %% Validate that the input is a SE3 matrix

    % Check if matrix is real
    assert( isreal(g), ...
        "Input is not a SE3 matrix: Matrix must be real." ...
        );

    % Check size
    assert( size(g,1) == 4 && size(g,2) == 4, ...
        "Input is not a SE3 matrix: Matrix must be 4x4." ...
        );

    % Check determinant
    assert( abs(det(g) - 1 ) < 10^-10, ...
        "Input is not a SE3 matrix: Matrix must have determinant 1." ...
        );

    % Check rotation matrix
    R = elara.SE3.matrix2Rx(g);
    elara.internal.validation.mustBeSO3Matrix(R);

    % Check last row
    assert( all(g(4,:) == [0,0,0,1]), ...
        "Input is not a SE3 matrix: Last row not [0 0 0 1]." ...
        );
end
