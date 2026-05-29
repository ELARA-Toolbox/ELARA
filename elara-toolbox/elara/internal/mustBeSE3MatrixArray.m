function mustBeSE3MatrixArray(g)
    %% Validate that the input is a array of SE3 matrices
    % where the array has dimensions (4,4,:,:...,:).
    % The first two dimensions are the rows and columns of the matrices;
    % the number of remaining dimensions is arbitrary.
    g = reshape(g, 4, 4, []);
    for iMat = 1:size(g,3)
        mustBeSE3Matrix(g(:,:,iMat));
    end
end