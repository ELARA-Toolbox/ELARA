function [q, u, qDot] = unpackNodeDecisionVariables(XVec, nSteps, nDoF, nInputs, opts)
    %% Unpack node variables and controls from a decision vector
    arguments
        % Decision vector with length (nDoF + nInputs)*(nSteps+1), or
        % (2*nDoF + nInputs)*(nSteps+1) when isODEDiscr is true
        XVec        (:,1)

        nSteps      (1,1) double

        % Number of node variables, or number of generalized coordinates
        % when isODEDiscr is true
        nDoF        (1,1) double
        nInputs     (1,1) double

        % Whether to return cell arrays instead of numerical matrices
        opts.cell   (1,1) logical = false;

        % Whether each node contains an ODE state x = [q; qDot], returned as
        % separate q and qDot outputs
        opts.isODEDiscr  (1,1) logical = false;
    end
    if opts.isODEDiscr
        nVarsStep  = 2*nDoF + nInputs;
    else
        nVarsStep  = nDoF + nInputs;
    end
    if opts.cell
        u  = cell(nSteps+1,1);
        q  = cell(nSteps+1,1);
        qDot = cell(nSteps+1,1);
    else
        u  = zeros(nInputs, nSteps+1);
        q  = zeros(nDoF,    nSteps+1);
        qDot = zeros(nDoF,   nSteps+1);
    end
    for k = 1:(nSteps+1)
        iX = (k-1)*nVarsStep+1:k*nVarsStep;
        iq  = iX((1:nDoF));
        if opts.isODEDiscr
            iqd = iX(nDoF+1:2*nDoF);
            iu  = iX(2*nDoF+1:end);
        else
            iu  = iX(nDoF+1:end);
        end
        if opts.cell
            q{k}  = XVec(iq);
            u{k}  = XVec(iu);
            if opts.isODEDiscr
                qDot{k} = XVec(iqd);
            end
        else
            q(:,k)  = XVec(iq);
            u(:,k)  = XVec(iu);
            if opts.isODEDiscr
                qDot(:,k) = XVec(iqd);
            end
        end
    end
end
