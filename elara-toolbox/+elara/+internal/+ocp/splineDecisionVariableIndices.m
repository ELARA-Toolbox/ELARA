function [indexMat_q, indexMat_z] = splineDecisionVariableIndices( ...
        nSteps, nDoF, nInputs, nInputSplinePoints, opts)
    %% Get indices of coordinates q and input spline control points z 
    % in overall vector of decision variables
    arguments
        nSteps      (1,1) double
        nDoF        (1,1) double
        nInputs     (1,1) double
        nInputSplinePoints (1,1) double

        % Whether the state variables in XVec consists of (q,qDot), which
        % is the case for ODE discretizations
        opts.isODEDiscr  (1,1) logical = false;
    end

    % Overall number of elements (=vectors of q or z)
    nElements = nSteps+1 + nInputSplinePoints;

    % Vector of all indices
    %indices_all = 1:nElements;

    % Get evenly spaced indices for the spline control points
    indices_z = round(linspace(1, nElements, nInputSplinePoints));

    % Get indices of q: Simply use remaining indices
    % indices_q = setdiff(indices_all, indices_z);

    % Now get the full indices into decision variable vector X
    indexMat_q = zeros(nDoF, nSteps+1);
    indexMat_z = zeros(nInputs, nInputSplinePoints);
    lastIndex = 0;
    lastColumn_z = 0;
    lastColumn_q = 0;
    for iElem = 1:nElements
        % Check whether the current element belongs to q or z and compute
        % indices
        if ismember(iElem, indices_z)
            indexMat_z(:,lastColumn_z+1) = (lastIndex+1):(lastIndex + nInputs);
            lastIndex = lastIndex + nInputs;
            lastColumn_z = lastColumn_z + 1;
        else
            indexMat_q(:,lastColumn_q+1) = (lastIndex+1):(lastIndex + nDoF);
            lastIndex = lastIndex + nDoF;
            lastColumn_q = lastColumn_q + 1;
        end
    end
end
