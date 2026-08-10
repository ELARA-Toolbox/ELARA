function [x_sol, u_sol, u_sol_z, sol, stats] = solve(OCP, xInit, uInit, opts)
    %% Solve an OCP with CasADi NLP solver
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        OCP         (1,1) elara.ocp.Problem

        % Initial guess configuration (VI) or state (ODE) trajectory
        xInit       (:,:) double

        % Initial control decision variables: time-node values for direct
        % parameterization or B-spline control points
        uInit       (:,:) double

        % Struct with the results of a previous solver run; if given, used
        % to initialize/warm-start the solver
        opts.solWarmStart    (1,1) struct = struct();
    end

    %% Decision variable bounds

    nSteps = OCP.nSteps;
    nInputs = OCP.systemSym.nInputs;
    nDoF = OCP.systemSym.nDoF;

    lb_x_nlp = -inf(size(xInit));
    ub_x_nlp = +inf(size(xInit));

    if ~isempty(OCP.qMin)
        lb_x_nlp(1:nDoF,:) = repmat(OCP.qMin, [1, size(xInit, 2)]);
    end
    if ~isempty(OCP.qMax)
        ub_x_nlp(1:nDoF,:) = repmat(OCP.qMax, [1, size(xInit, 2)]);
    end

    if OCP.useSplineInputs
        lb_z_nlp = -inf(size(uInit));
        ub_z_nlp = inf(size(uInit));
    else
        if isempty(OCP.uMin)
            lb_u_nlp = -inf(size(uInit));
        else
            lb_u_nlp = repmat(OCP.uMin, [1, size(uInit, 2)]);
        end
        if isempty(OCP.uMax)
            ub_u_nlp = inf(size(uInit));
        else
            ub_u_nlp = repmat(OCP.uMax, [1, size(uInit, 2)]);
        end

        % Initial values
        if ~isempty(OCP.u0)
            ub_u_nlp(:,1) = OCP.u0;
            lb_u_nlp(:,1) = OCP.u0;
        end
    end

    if OCP.discretization.type == "varint"
        if ~isempty(OCP.q0)
            lb_x_nlp(:,1) = OCP.q0;
            ub_x_nlp(:,1) = OCP.q0;
        end
        if 0%~isempty(OCP.qF)
            % Final time constraint for q currently implemented as explicit
            % constraint in the OCP solve function
            lb_q_nlp(:,end) = OCP.qF;
            ub_q_nlp(:,end) = OCP.qF;
        end
        if OCP.useSplineInputs
            lb_X = elara.internal.ocp.packSplineDecisionVariables(lb_x_nlp, lb_z_nlp);
            ub_X = elara.internal.ocp.packSplineDecisionVariables(ub_x_nlp, ub_z_nlp);
        else
            lb_X = elara.internal.ocp.packNodeDecisionVariables(lb_x_nlp, lb_u_nlp);
            ub_X = elara.internal.ocp.packNodeDecisionVariables(ub_x_nlp, ub_u_nlp);
        end
    else
        if ~isempty(OCP.q0)
            lb_x_nlp(1:nDoF,1) = OCP.q0;
            ub_x_nlp(1:nDoF,1) = OCP.q0;
        end
        if 0%~isempty(OCP.qF)
            lb_q_nlp(1:nDoF,end) = OCP.qF;
            ub_q_nlp(1:nDoF,end) = OCP.qF;
        end
        if ~isempty(OCP.qDot0)
            lb_x_nlp(nDoF+1:end,1) = OCP.qDot0;
            ub_x_nlp(nDoF+1:end,1) = OCP.qDot0;
        end
        if ~isempty(OCP.qDotF)
            lb_x_nlp(nDoF+1:end,end) = OCP.qDotF;
            ub_x_nlp(nDoF+1:end,end) = OCP.qDotF;
        end
        if OCP.useSplineInputs
            lb_X = elara.internal.ocp.packSplineDecisionVariables(lb_x_nlp, lb_z_nlp);
            ub_X = elara.internal.ocp.packSplineDecisionVariables(ub_x_nlp, ub_z_nlp);
        else
            lb_X = reshape([lb_x_nlp; lb_u_nlp], [], 1);
            ub_X = reshape([ub_x_nlp; ub_u_nlp], [], 1);
        end
    end

    %% Solve NLP

    % Prepare solver arguments
    p = struct;
    p.lbx = lb_X;
    p.ubx = ub_X;
    p.lbg = OCP.constrDef.lb_c;
    p.ubg = OCP.constrDef.ub_c;
    p.p = [OCP.runningCostWeights; OCP.finalCostWeights; OCP.x_TCP_F];
    if OCP.useSplineInputs
        p.x0 = elara.internal.ocp.packSplineDecisionVariables(xInit, uInit);
    else
        p.x0 = elara.internal.ocp.packNodeDecisionVariables(xInit, uInit);
    end

    % Warm-start solver with previous solution, if given
    if all(isfield(opts.solWarmStart, ["x", "lam_x", "lam_g"]))

        fprintf("\nInfo: Using warm-start for NLP solver.\n");
        p.x0     = opts.solWarmStart.x;
        p.lam_x0 = opts.solWarmStart.lam_x;
        p.lam_g0 = opts.solWarmStart.lam_g;
    end

    fprintf('Solving NLP...\n\n');
    sol = OCP.NLPSolver.call(p);
    stats = OCP.NLPSolver.stats;


    %% Extract solution

    if OCP.discretization.type == "varint"
        nStates = nDoF;
    else
        nStates = 2*nDoF;
    end
    if OCP.useSplineInputs
        [x_sol, u_sol_z] = elara.internal.ocp.unpackSplineDecisionVariables( ...
            full(sol.x), nSteps, nStates, nInputs, OCP.nInputSplinePoints);
    
        % Evaluate the control trajectory from the spline control points
        B = OCP.getInputSplineBasisMatrix;
        u_sol = (B*u_sol_z.').';
    else
        [x_sol, u_sol_z] = elara.internal.ocp.unpackNodeDecisionVariables( ...
            full(sol.x), nSteps, nStates, nInputs);
        u_sol = u_sol_z;
    end
end
