function [q, z, qd] = unpackSplineDecisionVariables(XVec, nSteps, nDoF, nInputs, nInputSplinePoints, opts)
    %% Convert vector of decision variables to q and z matrices
    arguments
        % Vector of decision variables, has length
        % (nDoF + nInputs)*(nSteps+1)
        XVec        (:,1)

        nSteps      (1,1) double
        nDoF        (1,1) double
        nInputs     (1,1) double
        nInputSplinePoints (1,1) double

        % Whether to give outputs as numerical matrices or cell arrays
        opts.cell   (1,1) logical = false;

        % Whether the state variables in XVec consists of (q,qDot), which
        % is the case for ODE discretizations
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

    % Extract q and z
    if opts.cell
        z  = cell(nInputSplinePoints,1);
        q  = cell(nSteps+1,1);
        qd = cell(nSteps+1,1);
        for iq = 1:(nSteps+1)
            if opts.isODEDiscr
                q{iq}  = XVec(indexMat_q(:,iq));
                qd{iq} = XVec(indexMat_qd(:,iq));
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
        qd = XVec(indexMat_qd);
    end
end
