function [c, lb_c, ub_c, g, c_dyn, c_WS] = OCPConstraintFunODE(OCP, x, u)
    %% Evaluate the constraint function c for a ODE discretization scheme
    % i.e., the function c(q,u) that defines the constraints c = 0
    arguments
        OCP     (1,1) OCPDefinition

        % Matrices of states and inputs at time steps
        x       (:,1) cell % (2*nDoF,  nSteps+1)
        u       (:,:) cell % (nInputs, nSteps+1)
        % For RK: u is matrix, where the rows contain the input values at
        % the stages of the RK method
    end

    %%
    MBSys = OCP.MBSys;
    nSteps = OCP.nSteps;
    simPars = OCP.simPars;
    h = OCP.h;

    % Get workspace constraint functions
    % NOTE:
    % Workspace constraints are currently not considered for the reference
    % implementations!
    %[dIntFun, dExtFun] = getCasadiPositionWorkspaceDistFuns(MBSys.nFrames, OCP.workSpaceDef);


    %% Define step constraint function

    % Define Casadi RHS Function
    xSym = casadi.MX.sym('x', 2*MBSys.nDoF, 1);
    uSym = casadi.MX.sym('x', MBSys.nInputs, 1);
    Fsym = computeFirstOrderSystemRHS_MInv_casadi(0, xSym, uSym, MBSys, simPars);
    FFun = casadi.Function('FFun', {xSym, uSym}, {Fsym});

    x_kSym  = casadi.MX.sym('x_k', 2*MBSys.nDoF, 1);
    x_k1Sym = casadi.MX.sym('x_k1', 2*MBSys.nDoF, 1);

    if OCP.useSplineInputs
        assert(size(u,2) == OCP.nSteps+1);
        u_kStageSym = casadi.MX.sym('u_kStage', MBSys.nInputs, size(u,1));
        eq_int = OCP.discretization.getIntegrationStepConstraintSpline(FFun, x_kSym, x_k1Sym, u_kStageSym, h);
        FStep = casadi.Function('FStep', {x_kSym, x_k1Sym, u_kStageSym}, {eq_int});
    else
        u_kSym  = casadi.MX.sym('u_k', MBSys.nInputs, 1);
        u_k1Sym = casadi.MX.sym('u_k1', MBSys.nInputs, 1);
        eq_int = OCP.discretization.getIntegrationStepConstraint(FFun, x_kSym, x_k1Sym, u_kSym, u_k1Sym, h);
        FStep = casadi.Function('FStep', {x_kSym, x_k1Sym, u_kSym, u_k1Sym}, {eq_int});
    end


    %% Evaluate constraints

    % Initialize empty arrays
    lb_c = cell(nSteps, 1);
    ub_c = cell(nSteps, 1);
    c     = cell(nSteps, 1);
    c_dyn = cell(nSteps, 1); % Holds the dynamics constraints at each time step
    c_WS  = cell(0,2);       % Holds the workspace constraints at each time step
    g    = cell(nSteps+1,1); % Frame configurations at each time step

    % All steps 1, ..., N
    for k = 1:nSteps
        if OCP.useSplineInputs
            c_dyn_k = FStep(x{k}, x{k+1}, horzcat(u{:,k}));
        else
            c_dyn_k = FStep(x{k}, x{k+1}, u{k}, u{k+1});
        end
        c{k}     = c_dyn_k;
        c_dyn{k} = c_dyn_k;
        lb_c{k} = zeros(2*MBSys.nDoF, 1);
        ub_c{k} = zeros(2*MBSys.nDoF, 1);
        g{k} = MBSys.computeFwdKin(x{k}(1:MBSys.nDoF));
    end

    % Kinematics final step
    g{end} = MBSys.computeFwdKin(x{nSteps+1}(1:MBSys.nDoF));


    % For spline input parameterization: Add input contraints
    % Todo: Not nice to do it here; better would be in the solveOCP
    % function, but there we don't have access to the decision variables
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        % Only enforce limits at the time nodes, not the stage values
        c_u = u(1,1:end-1).';

        if ~isempty(OCP.uMin)
            lb_u = repmat({OCP.uMin}, [OCP.nSteps,1]);
        else
            lb_u = repmat({-inf(MBSys.nInputs,1)}, [OCP.nSteps,1]);
        end
        if ~isempty(OCP.uMax)
            ub_u = repmat({OCP.uMax}, [OCP.nSteps,1]);
        else
            ub_u = repmat({inf(MBSys.nInputs,1)}, [OCP.nSteps,1]);
        end
        lb_c = [lb_c, lb_u];
        ub_c = [ub_c, ub_u];
    else
        c_u = cell(nSteps,0);
    end
    c = reshape([c_dyn, c_u].', [], 1);
    lb_c = reshape(lb_c.', [], 1);
    ub_c = reshape(ub_c.', [], 1);
    lb_c = vertcat(lb_c{:});
    ub_c = vertcat(ub_c{:});

    % Final constraint for u
    if OCP.useSplineInputs && (~isempty(OCP.uMin) || ~isempty(OCP.uMax))
        c{end+1} = u{1,end};
        if ~isempty(OCP.uMin)
            lb_u = OCP.uMin;
        else
            lb_u = -inf(MBSys.nInputs,1);
        end
        if ~isempty(OCP.uMax)
            ub_u = OCP.uMax;
        else
            ub_u = inf(MBSys.nInputs,1);
        end
        lb_c = [lb_c; lb_u];
        ub_c = [ub_c; ub_u];
    end

end
