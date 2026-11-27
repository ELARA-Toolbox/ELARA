function [qOpt, uOpt] = computeStaticEquilibriumTCPPos(OCP, opts)
    %% Compute a static equilibrium for the problem's desired TCP position
    % The equilibrium satisfies the static force balance, workspace
    % constraints, control bounds, and configured final TCP objective or
    % constraint of the optimal-control problem.
    arguments
        OCP (1,1) elara.ocp.Problem

        % Squared-norm weights of the optimization problem,
        % ordered as in Problem.finalCostWeights:
        % [controls, coordinates, TCP position error]
        opts.costWeights (3,1) double {mustBeFinite,mustBeNonnegative} = [1; 1e5; 1e8];

        % Clearance from workspace boundaries and obstacles, in meters.
        opts.safetyMargin (1,1) double {mustBeFinite,mustBeNonnegative} = 0.02;
    end

    %% Get variables
    systemSym = OCP.systemSym;
    simPars = OCP.simPars;

    % Verify that the TCP is defined for the system
    if ~systemSym.indexTCPFrame
        warning('elara:MissingTCPFrame', ...
            "No TCP frame defined in the ELARA System object. Using last system frame as the TCP frame.")
        indexTCPFrame = systemSym.nFrames;
    else
        indexTCPFrame = systemSym.indexTCPFrame;
    end

    % CasADi functions for signed workspace distances
    % Include the TCP position in workspace constraints only when a TCP is
    % explicitly defined.
    nFramesWS = systemSym.nFrames + double(systemSym.indexTCPFrame ~= 0);
    [dIntFun, dExtFun] = OCP.workspace.getSignedDistanceFunctions(nFramesWS);


    %% Define and solve optimization problem

    opti = casadi.Opti();
    q = opti.variable(systemSym.nDoF,1);
    u = opti.variable(systemSym.nInputs,1);

    % Statics/force balance constraint
    [res_statics, g] = elara.statics.sym.residual(systemSym, simPars, q, u);
    opti.subject_to( res_statics == 0 );

    % Workspace constraint
    g_TCP = g(indexTCPFrame) * elara.SE3.matrix2Element(systemSym.g_B_TCP);
    x_WS = horzcat(g.x);
    if systemSym.indexTCPFrame
        x_WS = [x_WS, g_TCP.x];
    end
    c_WSInt = dIntFun(x_WS);
    c_WSExt = dExtFun(x_WS);
    if length(c_WSInt) %#ok<ISMT>
        opti.subject_to(c_WSInt >= opts.safetyMargin);
    end
    if length(c_WSExt) %#ok<ISMT>
        opti.subject_to(c_WSExt <= -opts.safetyMargin);
    end

    % Control limits
    if ~isempty(OCP.uMin)
        opti.subject_to(u > OCP.uMin );
    end
    if ~isempty(OCP.uMax)
        opti.subject_to(u < OCP.uMax);
    end
    % TCP constraint
    if OCP.addTCPFinalTimeConstraint
        opti.subject_to( (g_TCP.x ) ==  OCP.x_TCP_F);
    end

    % Objective function
    J = opts.costWeights(1) * 1/2*sumsqr(u) ...
        + opts.costWeights(2) * 1/2 * sumsqr(q);

    if OCP.finalCostActive(end) || OCP.runningCostActive(end)
        J = J + opts.costWeights(3) * 1/2*sumsqr((OCP.x_TCP_F - g_TCP.x));
    end

    opti.minimize(J);

    opti.set_initial([q;u], zeros(systemSym.nDoF+systemSym.nInputs,1));

    p_opts = struct();
    s_opts = struct();
    p_opts.expand = true;
    opti.solver('ipopt', p_opts, s_opts);

    q0 = zeros(systemSym.nDoF,1);
    u0 = zeros(systemSym.nInputs,1);

    FOpti = opti.to_function('FOpti', {q,u}, {q,u});
    [qOpt, uOpt] = FOpti(q0, u0);

    qOpt = full(qOpt);
    uOpt = full(uOpt);

    % Make sure initial guess satisfies constraints exactly
    if ~isempty(OCP.uMin)
        uOpt = max(uOpt, OCP.uMin);
    end
    if ~isempty(OCP.uMax)
        uOpt = min(uOpt, OCP.uMax);
    end
end
