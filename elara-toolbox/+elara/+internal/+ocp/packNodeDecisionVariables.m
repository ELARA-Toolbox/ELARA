function XVec = packNodeDecisionVariables(x, u)
    %% Pack node variables and controls into a decision variable vector
    arguments
        % Configuration (VI) or state (ODE) values at all time nodes,
        % (nNodeVariables, nSteps+1)
        x       (:,:)

        % Control values at all time nodes, (nInputs, nSteps+1)
        u       (:,:)
    end
    nSteps  = size(x,2)-1;
    nNodeVariables = size(x,1);
    nInputs = size(u,1);

    assert(size(u,2)==nSteps+1);

    nVarsStep  = nNodeVariables + nInputs;
    nVarsTotal = (nSteps+1)*nVarsStep;
    XVec = zeros(nVarsTotal, 1);
    for k = 1:(nSteps+1)
        iX = (k-1)*nVarsStep+1:k*nVarsStep;
        ix = iX(1:nNodeVariables);
        iu = iX(nNodeVariables+1:end);
        XVec(ix) = x(:,k);
        XVec(iu) = u(:,k);
    end
end
