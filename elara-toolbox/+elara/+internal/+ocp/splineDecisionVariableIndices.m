function [indexMat_q, indexMat_z] = splineDecisionVariableIndices( ...
        nSteps, nDoF, nInputs, nInputSplinePoints)
    %% Get decision-vector indices of node variables and spline control points
    arguments
        nSteps      (1,1) double
        % Number of node variables stored at each time node
        nDoF        (1,1) double
        nInputs     (1,1) double
        nInputSplinePoints (1,1) double
    end

    % Total number of interleaved node-variable and control-point vectors
    nElements = nSteps+1 + nInputSplinePoints;

    % Get evenly spaced indices for the spline control points
    indices_z = round(linspace(1, nElements, nInputSplinePoints));

    % Construct the full indices into decision vector X
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
