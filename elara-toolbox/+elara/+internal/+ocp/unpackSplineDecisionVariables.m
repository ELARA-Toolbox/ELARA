function [q, z, qDot] = unpackSplineDecisionVariables(XVec, nSteps, nDoF, nInputs, nInputSplinePoints, opts)
    %% Unpack node variables and B-spline control points from a decision vector
    arguments
        % Decision vector with length
        % nDoF*(nSteps+1) + nInputs*nInputSplinePoints, or with 2*nDoF
        % node variables when isODEDiscr is true
        XVec        (:,1)

        nSteps      (1,1) double

        % Number of node variables, or number of generalized coordinates
        % when isODEDiscr is true
        nDoF        (1,1) double
        nInputs     (1,1) double
        nInputSplinePoints (1,1) double

        % Whether to return cell arrays instead of numerical matrices
        opts.cell   (1,1) logical = false;

        % Whether each node contains an ODE state x = [q; qDot], returned as
        % separate q and qDot outputs
        opts.isODEDiscr  (1,1) logical = false;
    end

    if opts.isODEDiscr
        [indexMat_x, indexMat_z] = elara.internal.ocp.splineDecisionVariableIndices( ...
            nSteps, 2*nDoF, nInputs, nInputSplinePoints);
        indexMat_q  = indexMat_x(1:nDoF,:);
        indexMat_qd = indexMat_x(nDoF+1:end,:);
    else
        [indexMat_q, indexMat_z] = elara.internal.ocp.splineDecisionVariableIndices( ...
            nSteps, nDoF, nInputs, nInputSplinePoints);
        indexMat_qd = [];
    end

    % Extract node variables and spline control points
    if opts.cell
        z  = cell(nInputSplinePoints,1);
        q  = cell(nSteps+1,1);
        qDot = cell(nSteps+1,1);
        for iq = 1:(nSteps+1)
            if opts.isODEDiscr
                q{iq}  = XVec(indexMat_q(:,iq));
                qDot{iq} = XVec(indexMat_qd(:,iq));
            else
                q{iq} = XVec(indexMat_q(:,iq));
            end
        end
        for iz = 1:nInputSplinePoints
            z{iz} = XVec(indexMat_z(:,iz));
        end
    else
        q = XVec(indexMat_q);
        z = XVec(indexMat_z);
        qDot = XVec(indexMat_qd);
    end
end
