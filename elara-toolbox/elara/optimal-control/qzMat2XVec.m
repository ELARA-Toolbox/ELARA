function XVec = qzMat2XVec(q, z)
    %% Convert matrices of inputs and configurations q, u to vector of decision variables
    arguments
        % Matrix of configurations at all time nodes
        % dimensions (nDoF, nSteps+1)
        q       (:,:)
        % Matrix of input parameters
        % dimensions (nInputs, nSplinePoints)
        z       (:,:)
    end
    nSteps  = size(q,2)-1;
    nDoF    = size(q,1);
    nInputs = size(z,1);
    nInputSplinePoints = size(z,2);

    [indexMat_q, indexMat_z] = getXVecIndicesSpline(nSteps, nDoF, nInputs, nInputSplinePoints);

    nVarsTotal =  nDoF*(nSteps+1) + nInputSplinePoints*nInputs;
    XVec = zeros(nVarsTotal, 1);
    XVec(indexMat_q(:)) = q(:);
    XVec(indexMat_z(:)) = z(:);
end