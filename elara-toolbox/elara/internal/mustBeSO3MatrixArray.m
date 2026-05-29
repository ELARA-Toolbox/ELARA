function mustBeSO3MatrixArray(R)
    %% Validate that the input is a array of SO3 matrices
    % where the array has dimensions (3,3,:,:...,:).
    % The first two dimensions are the rows and columns of the matrices;
    % the number of remaining dimensions is arbitrary.
    g = reshape(R, 3, 3, []);
    for iMat = 1:size(R,3)
        mustBeSO3Matrix(R(:,:,iMat));
    end
end