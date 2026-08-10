function [c, lb_c, ub_c, g, c_dyn, c_WS] = constraintFunODE(OCP, x, u)
    %% Evaluate the constraint function for an ODE discretization
    % i.e., the function c(x,u) containing the dynamics equality constraints
    % and workspace inequalities at the state nodes.
    arguments
        OCP     (1,1) elara.ocp.Problem

        % State vectors at the time nodes. Each cell contains a
        % (2*nDoF)-by-1 vector.
        x       (:,1) cell

        % Control vectors at the time nodes and, for Runge-Kutta methods
        % with spline controls, at the intermediate stages. Each cell
        % contains an nInputs-by-1 vector.
        u       (:,:) cell
    end

    %%
    system = OCP.systemSym;
    nSteps = OCP.nSteps;
    simPars = OCP.simPars;
    h = OCP.h;

    % Get workspace constraint functions. Include the TCP position when a
    % TCP transformation is defined for the system.
    if system.indexTCPFrame
        nFramesWS = system.nFrames + 1;
        g_B_TCP = elara.SE3.matrix2Element(system.g_B_TCP);
    else
        nFramesWS = system.nFrames;
        g_B_TCP = elara.SE3.Element;
    end
    [dIntFun, dExtFun] = OCP.workspace.getSignedDistanceFunctions(nFramesWS);


    %% Define step constraint function

    % Define the CasADi right-hand-side function
    xSym = casadi.MX.sym('x', 2*system.nDoF, 1);
    uSym = casadi.MX.sym('u', system.nInputs, 1);
    Fsym = elara.dynamics.sym.firstOrderDerivative(0, xSym, uSym, system, simPars);
    FFun = casadi.Function('FFun', {xSym, uSym}, {Fsym});

    x_kSym  = casadi.MX.sym('x_k', 2*system.nDoF, 1);
    x_k1Sym = casadi.MX.sym('x_k1', 2*system.nDoF, 1);

    if OCP.useSplineInputs
        assert(size(u,2) == OCP.nSteps+1);
        u_kStageSym = casadi.MX.sym('u_kStage', system.nInputs, size(u,1));
        eq_int = OCP.discretization.getIntegrationStepConstraintSpline(FFun, x_kSym, x_k1Sym, u_kStageSym, h);
        FStep = casadi.Function('FStep', {x_kSym, x_k1Sym, u_kStageSym}, {eq_int});
    else
        u_kSym  = casadi.MX.sym('u_k', system.nInputs, 1);
        u_k1Sym = casadi.MX.sym('u_k1', system.nInputs, 1);
        eq_int = OCP.discretization.getIntegrationStepConstraint(FFun, x_kSym, x_k1Sym, u_kSym, u_k1Sym, h);
        FStep = casadi.Function('FStep', {x_kSym, x_k1Sym, u_kSym, u_k1Sym}, {eq_int});
    end


    %% Evaluate constraints

    % Initialize empty arrays
    % Bounds: column 1 = dynamics, column 2 = workspace interior,
    % column 3 = workspace exterior/obstacles.
    lb_c = cell(nSteps, 3);
    ub_c = cell(nSteps, 3);
    c_dyn = cell(nSteps, 1); % Holds the dynamics constraints at each time step
    c_WS  = cell(nSteps+1,2); % Workspace constraints (interior and exterior)
    g    = cell(nSteps+1,1); % Frame configurations at each time step

    % All steps 1, ..., N
    for k = 1:nSteps
        if OCP.useSplineInputs
            c_dyn_k = FStep(x{k}, x{k+1}, horzcat(u{:,k}));
        else
            c_dyn_k = FStep(x{k}, x{k+1}, u{k}, u{k+1});
        end
        c_dyn{k} = c_dyn_k;
        lb_c{k,1} = zeros(2*system.nDoF, 1);
        ub_c{k,1} = zeros(2*system.nDoF, 1);

        g_k = system.computeFwdKin(x{k}(1:system.nDoF));
        g{k} = g_k;

        % Workspace constraints at the current state node.
        if system.indexTCPFrame
            g_k_TCP = g_k(system.indexTCPFrame) * g_B_TCP;
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

    % Kinematics and workspace constraints at the final state node.
    g_N = system.computeFwdKin(x{nSteps+1}(1:system.nDoF));
    g{end} = g_N;
    if system.indexTCPFrame
        g_N_TCP = g_N(system.indexTCPFrame) * g_B_TCP;
        x_N_WS = [[g_N.x], g_N_TCP.x];
    else
        x_N_WS = [g_N.x];
    end

    c_WSInt_N = dIntFun(x_N_WS);
    c_WSExt_N = dExtFun(x_N_WS);
    c_WS{end,1} = c_WSInt_N;
    c_WS{end,2} = c_WSExt_N;


    % For spline input parameterization: Add input constraints
    % Todo: Not nice to do it here; better would be in the solve function
    % function, but there we don't have access to the decision variables
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        % Only enforce limits at the time nodes, not the stage values
        c_u = u(1,1:end-1).';

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
    c = reshape([c_dyn, c_WS(1:nSteps,:), c_u].', [], 1);
    lb_c = reshape(lb_c.', [], 1);
    ub_c = reshape(ub_c.', [], 1);
    lb_c = vertcat(lb_c{:});
    ub_c = vertcat(ub_c{:});

    % Append workspace constraints at the final state node
    c = [c; {c_WSInt_N; c_WSExt_N}];
    lb_c = [lb_c; zeros(size(c_WSInt_N)); -inf(size(c_WSExt_N))];
    ub_c = [ub_c; inf(size(c_WSInt_N)); zeros(size(c_WSExt_N))];

    % Control limit at the final time node
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        c{end+1} = u{1,end};
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
