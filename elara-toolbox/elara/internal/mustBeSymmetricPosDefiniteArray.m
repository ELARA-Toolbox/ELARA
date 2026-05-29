function mustBeSymmetricPosDefiniteArray(A)
    %% Validate that the input is an array of positive semi-definite matrices
    dim = size(A,1);
    A = reshape(A, dim, dim, []);

    for iMat = 1:size(A,3)
        mustBeSymmetricPosDefinite(A(:,:,iMat));
    end
end