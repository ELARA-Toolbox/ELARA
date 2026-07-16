function [qOpt, uOpt] = computeOptimalSteadyStateInputsTCPPos(MBSys, OCP, simPars)
    %% Compute optimal steady-state system inputs for given TCP position
    arguments
        MBSys    (1,1) elara.internal.System
        OCP      (1,1) OCPDefinition
        simPars  (1,1) elara.SimulationParameters
    end

    %% Get variables
    if ~isa(MBSys, "elara.SystemSym")
        MBSysSym = convertelara.SystemObject(MBSys, "elara.SystemSym");
    else
        MBSysSym = MBSys;
    end

     % Verify that the TCP is defined for the system
    if ~MBSys.indexTCPFrame
        warning("No TCP frame defined in the elara.internal.System object. Using last frame as the TCP frame.")
        indexTCPFrame = MBSys.nFrames;
    else
        indexTCPFrame = MBSys.indexTCPFrame;
    end

    % Casadi function for signed workspace distance function
    [dIntFun, dExtFun] = getCasadiPositionWorkspaceDistFuns(MBSys.nFrames, OCP.workSpaceDef);


    %% Define and solve optimization problem

    opti = casadi.Opti();
    q = opti.variable(MBSysSym.nDoF,1);
    u = opti.variable(MBSysSym.nInputs,1);


    % Statics/force balance constraint
    %[res_statics, x] = computeStaticResiduum_casadi(sysFuns, q, u);
    [res_statics, g] = computeStaticResiduum_casadi(MBSysSym, simPars, q, u);
    opti.subject_to( res_statics == 0 );

    % Workspace constraint
    safetyMargin = 0.02;

    c_WSInt = dIntFun(horzcat(g.x));
    c_WSExt = dExtFun(horzcat(g.x));
    if length(c_WSInt) %#ok<ISMT>
        opti.subject_to(c_WSInt > safetyMargin);
    end
    if length(c_WSExt) %#ok<ISMT>
        opti.subject_to(c_WSExt < safetyMargin);
    end

    % Input limits
    if ~isempty(OCP.uMin)
        opti.subject_to(u > OCP.uMin );
    end
    if ~isempty(OCP.uMax)
        opti.subject_to(u < OCP.uMax);
    end
    % TCP constraint
    g_TCP = g(indexTCPFrame) * SE3(MBSys.g_B_TCP(1:3,1:3), MBSys.g_B_TCP(1:3,4));
    if OCP.addTCPFinalTimeConstraint
        opti.subject_to( (g_TCP.x ) ==  OCP.x_TCP_F);
    end

    % Objective function
    a1 = 1e8;
    a2 = 1;
    a3 = 100000;
    
    % % For SRFRobot
    % a1 = 1e5;
    % a2 = 1;
    % a3 = 100;

    J = a2 * 1/2*sumsqr(u) ...
        + a3 * 1/2 * sumsqr(q);

    if OCP.iFC(end) || OCP.iRC(end)
        J = J + a1 * 1/2*sumsqr((OCP.x_TCP_F - g_TCP.x));
    end

    opti.minimize(J);

    opti.set_initial([q;u], zeros(MBSysSym.nDoF+MBSysSym.nInputs,1));

    p_opts = struct();
    s_opts = struct();
    p_opts.expand = true;
    %s_opts.max_iter = 100;
    %s_opts.fixed_variable_treatment = 'relax_bounds';

    opti.solver('ipopt', p_opts, s_opts);

    q0 = zeros(MBSysSym.nDoF,1);
    u0 = zeros(MBSysSym.nInputs,1);

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