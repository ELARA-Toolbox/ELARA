function [c, lb_c, ub_c, g, c_DEL, c_WS] = constraintFunDEL(OCP, q, u, opts)
    %% Evaluate the constraint function c
    % i.e., the function c(q,u) that defines the constraints c = 0
    arguments
        OCP     (1,1) elara.ocp.Problem

        % Configuration vectors at the time nodes. Each cell contains an
        % nDoF-by-1 vector.
        q       (:,1) cell

        % Control vectors at the time nodes. Each cell contains an
        % nInputs-by-1 vector.
        u       (:,1) cell

        % Use a CasADi function to evaluate the DEL at each time step
        opts.useCasadiStepFunctions    (1,1) logical = false;
    end

    %% Get variables
    system = OCP.systemSym;
    nSteps = OCP.nSteps;
    simPars = OCP.simPars;

    % Weighting factor for generalized trapezoidal rule
    % Rectangle rule: a = 0, trapezoidal rule: a = 1/2
    aFirstLast = 1/2;
    aStep = OCP.discretization.aTrapez;

    h = OCP.h;

    % Get workspace constraint functions
    % Include TCP position in workspace constraints if TCP is defined
    if system.indexTCPFrame
        % Number of frames included in the workspace constraints
        nFramesWS = system.nFrames + 1;

        % TCP transformation as an SE(3) element
        g_B_TCP = elara.SE3.matrix2Element(OCP.systemSym.g_B_TCP);
    else
        nFramesWS = system.nFrames;
        g_B_TCP = elara.SE3.Element;
    end
    
    [dIntFun, dExtFun] = OCP.workspace.getSignedDistanceFunctions(nFramesWS);


    %% Define Function for step constraints
    if opts.useCasadiStepFunctions
        q_k0Sym = casadi.MX.sym('q_k0', system.nDoF, 1);
        q_kSym  = casadi.MX.sym('q_k', system.nDoF, 1);
        q_k1Sym = casadi.MX.sym('q_k1', system.nDoF, 1);
        u_kSym  = casadi.MX.sym('u_k', system.nInputs, 1);

        DEL_res_k_sym = elara.dynamics.sym.DELResidual(system, simPars, q_k0Sym, q_kSym, q_k1Sym, u_kSym, h, aStep);
        stepFun = casadi.Function('FStep', {q_k0Sym, q_kSym, q_k1Sym, u_kSym }, {DEL_res_k_sym});
    end

    %% Initialize empty arrays
    % preallocate steps 1...nSteps; last step will be added separately
    lb_c = cell(nSteps,3);  % Bounds: Col. 1 = DEL, Col. 2 = WS Int., Col. 3 = WS Ext.
    ub_c = cell(nSteps,3);
    c_DEL = cell(nSteps,1); % DEL constraints at each time step
    c_WS  = cell(nSteps,2); % Workspace constraints at each time step (Int and Ext)
    g   = cell(nSteps+1,1); % Frame configurations at each time step


    %% Initial step (k,k+1) = (0,1) (indices (1,2))
    [g_k, g_rel_k] = system.computeFwdKin(q{1});
    [g_k1, g_rel_k1] = system.computeFwdKin(q{2});
    eta_k = system.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);

    c_DEL{1} = elara.dynamics.sym.DELResidualInitialStep_noKinematics( system, simPars, ...
        q{1}, q{2}, g_k, g_rel_k, eta_k, u{1}, OCP.qDot0, h, aFirstLast);

    lb_c{1,1} = zeros(system.nDoF, 1);
    ub_c{1,1} = zeros(system.nDoF, 1);
    g{1} = g_k;

    % Initial workspace constraints
    % Get frame positions for the workspace constraints
    % (with/without TCP frame)
    if system.indexTCPFrame
        g_k_TCP = g_k(system.indexTCPFrame)*g_B_TCP;
        x_k_WS = [[g_k.x], g_k_TCP.x];
    else
        x_k_WS = [g_k.x];
    end

    c_WSInt_1 = dIntFun(x_k_WS);
    c_WSExt_1 = dExtFun(x_k_WS);

    c_WS{1,1} = c_WSInt_1;
    lb_c{1,2} = zeros(size(c_WSInt_1));
    ub_c{1,2} = inf(size(c_WSInt_1));

    c_WS{1,2} = c_WSExt_1;
    lb_c{1,3} = -inf(size(c_WSExt_1));
    ub_c{1,3} = zeros(size(c_WSExt_1));

    % Placeholder values for external forces
    f_frame_k_b_ext = zeros(6, system.nFrames);
    f_frame_k_s_ext = zeros(6, system.nFrames);


    %% Intermediate steps k = 1, ..., N-1 (indices 2, ..., nSteps)
    for k = 2:nSteps
        eta_k0  = eta_k;
        g_k     = g_k1;
        g_rel_k = g_rel_k1;

        [g_k1, g_rel_k1] = system.computeFwdKin(q{k+1});
        eta_k = system.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);

        if opts.useCasadiStepFunctions
            c_DEL{k} = stepFun(q{k-1}, q{k}, q{k+1}, u{k});
        else
            c_DEL{k} = elara.dynamics.sym.DELResidual_noKinematics( ...
                system, simPars, q{k-1}, q{k}, q{k+1}, g_k, g_rel_k, ...
                eta_k, eta_k0, u{k}, f_frame_k_b_ext, f_frame_k_s_ext, h, aStep);
        end

        lb_c{k,1} = zeros(system.nDoF, 1);
        ub_c{k,1} = zeros(system.nDoF, 1);
        g{k} = g_k;

        % Step workspace constraints
        if system.indexTCPFrame
            g_k_TCP = g_k(system.indexTCPFrame)*g_B_TCP;
            x_k_WS = [[g_k.x], g_k_TCP.x];
        else
            x_k_WS = [g_k.x];
        end
        c_WSInt_k = dIntFun(x_k_WS);
        c_WSExt_k = dExtFun(x_k_WS);

        c_WS{k,1} = c_WSInt_k;
        lb_c{k,2} = zeros(size(c_WSInt_k));
        ub_c{k,2} = inf(size(c_WSInt_k));
        c_WS{k,2} = c_WSExt_k;
        lb_c{k,3} = -inf(size(c_WSExt_k));
        ub_c{k,3} = zeros(size(c_WSExt_k));
    end

    g{end} = system.computeFwdKin(q{nSteps+1});


    % For spline input parameterization: Add input constraints
    % Todo: Not nice to do it here; better would be in the solve function
    % function, but there we don't have access to the decision variables
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        c_u = u(1:end-1);

        if ~isempty(OCP.uMin)
            lb_u = repmat({OCP.uMin}, [OCP.nSteps,1]);
        else
            lb_u = repmat({-inf(system.nInputs,1)}, [OCP.nSteps,1]);
        end
        if ~isempty(OCP.uMax)
            ub_u = repmat({OCP.uMax}, [OCP.nSteps,1]);
        else
            ub_u = repmat({inf(system.nInputs,1)}, [OCP.nSteps,1]);
        end
        lb_c = [lb_c, lb_u];
        ub_c = [ub_c, ub_u];
    else
        c_u = cell(nSteps,0);
    end

    c = reshape([c_DEL, c_WS, c_u].', [], 1);
    lb_c = reshape(lb_c.', [], 1);
    ub_c = reshape(ub_c.', [], 1);
    lb_c = vertcat(lb_c{:});
    ub_c = vertcat(ub_c{:});


    %% Final step (k,k+1) = (N-1,N) (indices (nSteps, nSteps+1))
    if ~isempty(OCP.qDotF)
        g_rel_N  = system.computeJointTransformations(q{nSteps});
        [g_N1, g_rel_N1] = system.computeFwdKin(q{nSteps+1});
        eta_N = system.computeDiscreteAbsoluteVelocities(g_rel_N, g_rel_N1, h);

        c_DEL_N1 = elara.dynamics.sym.DELResidualFinalStep_noKinematics( system, simPars, ...
            q{nSteps}, q{nSteps+1}, g_N1, g_rel_N1, ...
            eta_N, u{nSteps+1}, OCP.qDotF, h, aFirstLast);

        c = [c; {c_DEL_N1}];
        c_DEL = [c_DEL; {c_DEL_N1}];
        lb_c = [lb_c; zeros(size(c_DEL_N1))];
        ub_c = [ub_c; zeros(size(c_DEL_N1))];
    else
        % No final velocity boundary condition: Last u_N1 is not assigned!
        % Simply set equal to u_N...
        c = [c; {u{nSteps+1}-u{nSteps}}];
        lb_c = [lb_c; zeros(system.nInputs,1)];
        ub_c = [ub_c; zeros(system.nInputs,1)];
    end

    % Final workspace constraint
    if system.indexTCPFrame
        g_k1_TCP = g_k1(system.indexTCPFrame)*g_B_TCP;
        x_k1_WS = [[g_k1.x], g_k1_TCP.x];
    else
        x_k1_WS = [g_k1.x];
    end
    c_WSInt_k = dIntFun(x_k1_WS);
    c_WSExt_k = dExtFun(x_k1_WS);
    c = [c; {c_WSInt_k; c_WSExt_k}];
    lb_c = [lb_c; zeros(size(c_WSInt_k)); -inf(size(c_WSExt_k))];
    ub_c = [ub_c; inf(size(c_WSInt_k));  zeros(size(c_WSExt_k))];
    c_WS = [c_WS; {c_WSInt_k, c_WSExt_k}];

    % Control limit at the final time node
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        c{end+1} = u{end};
        if ~isempty(OCP.uMin)
            lb_u = OCP.uMin;
        else
            lb_u = -inf(system.nInputs,1);
        end
        if ~isempty(OCP.uMax)
            ub_u = OCP.uMax;
        else
            ub_u = inf(system.nInputs,1);
        end
        lb_c = [lb_c; lb_u];
        ub_c = [ub_c; ub_u];
    end
end
