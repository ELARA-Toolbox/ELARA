function XVec = packNodeDecisionVariables(q, u)
    %% Convert matrices of inputs and configurations q, u to vector of decision variables
    arguments
        % Matrix of configurations at all time nodes
        % dimensions (nDoF, nSteps+1)
        q       (:,:)
        % Matrix of inputs at all time nodes
        % dimensions (nInputs, nSteps+1)
        u       (:,:)
    end
    nSteps  = size(q,2)-1;
    nDoF    = size(q,1);
    nInputs = size(u,1);

    assert(size(u,2)==nSteps+1);

    nVarsStep  = nDoF + nInputs;
    nVarsTotal = (nSteps+1)*nVarsStep;
    XVec = zeros(nVarsTotal, 1);
    for k = 1:(nSteps+1)
        iX = (k-1)*nVarsStep+1:k*nVarsStep;
        iq = iX((1:nDoF));
        iu = iX(nDoF+1:end);
        XVec(iq) = q(:,k);
        XVec(iu) = u(:,k);
    end
end
