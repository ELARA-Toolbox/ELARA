function XVec = packSplineDecisionVariables(x, z)
    %% Pack node variables and B-spline control points into a decision variable vector
    arguments
        % Configuration (VI) or state (ODE) values at all time nodes,
        % (nNodeVariables, nSteps+1)
        x       (:,:)

        % B-spline control points, (nInputs, nSplinePoints)
        z       (:,:)
    end
    nSteps  = size(x,2)-1;
    nNodeVariables = size(x,1);
    nInputs = size(z,1);
    nInputSplinePoints = size(z,2);

    [indexMat_x, indexMat_z] = elara.internal.ocp.splineDecisionVariableIndices( ...
        nSteps, nNodeVariables, nInputs, nInputSplinePoints);

    nVarsTotal = nNodeVariables*(nSteps+1) + nInputSplinePoints*nInputs;
    XVec = zeros(nVarsTotal, 1);
    XVec(indexMat_x(:)) = x(:);
    XVec(indexMat_z(:)) = z(:);
end
