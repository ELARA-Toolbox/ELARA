function [NLPSolver, constrDef] = defineOCPSolver(OCP, opts)
    %% Define an OCP with CasADi NLP solver
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        OCP     (1,1) OCPDefinition

        % Use a casadi function to evaluate the DEL in each time step
        opts.useCasadiStepFunctions (1,1) logical = false;

        % Draw debug plots (constraint Jacobian etc.)?
        opts.showDebugPlots (1,1) logical = false;
    end

    %% Initial checks
    nInputs = OCP.system.nInputs;
    nDoF = OCP.system.nDoF;

    % Check end time and time step and display warning if the time step is
    % not an even divisor of the end time (important for simulation studies)
    if rem(OCP.tF, OCP.h*OCP.nSteps)
        warning("OCP time step (h=%.3e) not an even divisor of the end time (%.3f)!", OCP.h, OCP.tEnd);
    end

    % Verify that the TCP is defined for the system
    if ~OCP.system.indexTCPFrame
        warning("No TCP frame defined in the elara.internal.System object. Using last frame as the TCP frame.")
        indexTCPFrame = OCP.system.nFrames;
    else
        indexTCPFrame = OCP.system.indexTCPFrame;
    end

    % Verify that TCP trajectory is specified if the TCP tracking cost is
    % active
    if OCP.iRC(5)
        assert(~isempty(OCP.x_TCP_traj), ...
            "No TCP trajectory specified but TCP tracking cost active.");
        assert(size(OCP.x_TCP_traj,2) == OCP.nSteps+1, ...
            "TCP trajectory has wrong dimensions (nr. of time steps).");
    end


    %% Prepare NLP variables

    nSteps = OCP.nSteps;
    if OCP.discretization.type == "varint"
        if OCP.useSplineInputs
            X_nlp = casadi.MX.sym('x', nDoF*(nSteps+1) + OCP.nInputSplinePoints*nInputs, 1);
            [q_C, z_C] = XVec2qzMat( X_nlp, OCP.nSteps, nDoF, ...
                nInputs, OCP.nInputSplinePoints, "cell", true);
        else
            nVarsStep  = nDoF + nInputs;
            X_nlp = casadi.MX.sym('x', nVarsStep*(nSteps+1), 1);
            [q_C, u_C] = XVec2quMat(X_nlp, nSteps, nDoF, ...
                nInputs, "cell", true);
        end
        x_C = q_C; % System states at each time step
    else
        if OCP.useSplineInputs
            X_nlp = casadi.MX.sym('x', 2*nDoF*(nSteps+1) + OCP.nInputSplinePoints*nInputs, 1);
            x_C = XVec2qzMat( X_nlp, OCP.nSteps, 2*nDoF, ...
                nInputs, OCP.nInputSplinePoints, "cell", true, "isODEDiscr", false);
            [q_C, z_C, ~] = XVec2qzMat( X_nlp, OCP.nSteps, nDoF, ...
                nInputs, OCP.nInputSplinePoints, "cell", true, "isODEDiscr", true);
        else
            nVarsStep  = 2*nDoF + nInputs;
            X_nlp = casadi.MX.sym('x', nVarsStep*(nSteps+1), 1);
            x_C = XVec2quMat(X_nlp, nSteps, 2*nDoF, nInputs, "cell", true, "isODEDiscr", false);
            [q_C, u_C, ~] = XVec2quMat(X_nlp, nSteps, nDoF, nInputs, "cell", true, "isODEDiscr", true);
        end
    end

    % Compute input variables and derivatives for B-spline parameterization
    if OCP.useSplineInputs
        % Prepare stage values of the input variables
        % (= sub-steps in fractions of h)
        stageVals = OCP.discretization.timeStepStageValues;

        % Get input values at time step
        [B, B_dt, B_ddt] = OCP.getInputSplineBasisMatrix("stageValues", stageVals);
        u_C = cell(size(B,1),1);
        ud_C = cell(size(B,1),1);
        udd_C = cell(size(B,1),1);
        for iStep = 1:size(B,1)
            for iCP = 1:OCP.nInputSplinePoints
                if any(B(iStep,iCP))
                    if isempty(u_C{iStep})
                        u_C{iStep}   = B(iStep, iCP) * z_C{iCP};
                        ud_C{iStep}  = B_dt(iStep, iCP) * z_C{iCP};
                        udd_C{iStep} = B_ddt(iStep, iCP) * z_C{iCP};
                    else
                        u_C{iStep}   = u_C{iStep}   + B(iStep, iCP) * z_C{iCP};
                        ud_C{iStep}  = ud_C{iStep}  + B_dt(iStep, iCP) * z_C{iCP};
                        udd_C{iStep} = udd_C{iStep} + B_ddt(iStep, iCP) * z_C{iCP};
                    end
                end
            end
        end
        % Rearrange vector of inputs (containing input values and
        % additional stage value times) into matrix
        sRK = length(stageVals);
        u_C   = reshape(u_C, sRK, OCP.nSteps+1);
        ud_C  = reshape(ud_C, sRK, OCP.nSteps+1);
        udd_C = reshape(udd_C, sRK, OCP.nSteps+1);
        u   = horzcat(u_C{:});
        ud  = horzcat(ud_C{:});
        udd = horzcat(udd_C{:});
    else
        u = horzcat(u_C{:});
        [ud, udd] = diffHigherOrder(u, OCP.h, OCP.FDOrder);
    end
    q = horzcat(q_C{:});
    [~, qdd]  = diffHigherOrder(q, OCP.h, OCP.FDOrder);


    %% Define constraints
    fprintf("Constructing constraint function... ");
    tic;

    if OCP.discretization.type == "varint"
        [c, lb_c, ub_c, g, c_DEL, c_WS] = OCPConstraintFunDEL(OCP, x_C, u_C, ...
            "useCasadiStepFunctions", opts.useCasadiStepFunctions);
    else
        [c, lb_c, ub_c, g, c_DEL, c_WS] = OCPConstraintFunODE(OCP, x_C, u_C);
    end
    g_F = g{end};

    % TCP constraint at final time
    g_TCP_F = g_F(indexTCPFrame) * SE3(OCP.system.g_B_TCP(1:3,1:3), OCP.system.g_B_TCP(1:3,4));
    if OCP.addTCPFinalTimeConstraint
        fprintf("\n   Adding final time TCP position constraint...\n")
        c = [c; {g_TCP_F.x - OCP.x_TCP_F}];
        lb_c = [lb_c; zeros(3,1)];
        ub_c = [ub_c; zeros(3,1)];
    end

    % Configuration constraint at final time
    % (explicit constraint seems to lead to better convergence than
    % incorporating it via variable bounds)
    if ~isempty(OCP.qF)
        fprintf("\n   Adding final time configuration constraint...\n")
        c = [c; q_C(end)];
        lb_c = [lb_c; OCP.qF];
        ub_c = [ub_c; OCP.qF];
    end

    c = vertcat(c{:});
    tMeas = toc;
    fprintf("took %.3f s.\n", tMeas);

    % Plot constraint jacobian for debugging
    if opts.showDebugPlots
        figure("Name", "Jacobian Constraints", "NumberTitle", "off");
        spy(jacobian(c, X_nlp),5)
        title("Constraint Jacobian")
        xlabel("decision variables $x_j$", "Interpreter", "latex");
        ylabel("constraint function $c_i(x)$", "Interpreter", "latex");
        drawnow;
    end

    %% Define cost function

    fprintf('Constructing objective function... ');
    tic;

    %%% Running cost
    wR = casadi.MX.sym('wR', 5,1);
    JR = cell(5,1);

    % TPC trajectory tracking cost
    J_TCP_tr = casadi.MX.zeros(1,1);
    if OCP.iRC(5)
        for iStep = 1:OCP.nSteps + 1
            g_TCP_k = g{iStep}(indexTCPFrame) * SE3(OCP.system.g_B_TCP(1:3,1:3), OCP.system.g_B_TCP(1:3,4));

            % Factor for trapezoidal rule integration
            % First and last step have factor 1/2
            fTrap = 1 - 1/2*(iStep == 1 || iStep == (OCP.nSteps+1));

            J_TCP_tr = J_TCP_tr + wR(5) * OCP.h * fTrap * 0.5*sumsqr(OCP.x_TCP_traj(:,iStep) - g_TCP_k.x);
        end
    end
    JR{5} = J_TCP_tr;

    if OCP.useSplineInputs
        JR{1} = wR(1) * OCP.discretization.integrateCostFunctionValueSpline(OCP, u, u_C);
        JR{2} = wR(2) * OCP.discretization.integrateCostFunctionValueSpline(OCP, ud, ud_C);
        JR{3} = wR(3) * OCP.discretization.integrateCostFunctionValueSpline(OCP, udd, udd_C);
    else
        JR{1} = wR(1) * OCP.discretization.integrateCostFunctionValue(OCP, u);
        JR{2} = wR(2) * OCP.discretization.integrateCostFunctionValue(OCP, ud);
        JR{3} = wR(3) * OCP.discretization.integrateCostFunctionValue(OCP, udd);
    end
    JR{4} = wR(4) * OCP.discretization.integrateCostFunctionValue(OCP, qdd);

    %%% Final cost
    wF = casadi.MX.sym('wF', 3,1);
    JF = cell(3,1);
    x_TCP_F = casadi.MX.sym('x_TCP', 3,1);
    JF{1} = wF(1)*0.5*sumsqr(u(:,end));
    JF{2} = wF(2)*0.5*sumsqr(q(:,end));
    JF{3} = wF(3)*0.5*sumsqr(x_TCP_F - g_TCP_F.x);

    %%% Add together
    J = casadi.MX.zeros(1,1);
    for iJR = 1:5
        if OCP.iRC(iJR)
            J = J + JR{iJR};
        end
    end
    for iJF = 1:3
        if OCP.iFC(iJF)
            J = J + JF{iJF};
        end
    end

    tMeas = toc;
    fprintf("took %.3f s.\n", tMeas);

    %% Assign to constraint definition struct

    constrDef = struct;
    constrDef.lb_c = lb_c;
    constrDef.ub_c = ub_c;
    constrDef.Fun_c = casadi.Function('c', ...
        {X_nlp, x_TCP_F}, {c}, {'x_nlp', 'x_TCP_F'}, {'c'} ...
        );
    constrDef.Fun_cDyn = casadi.Function('cDEL', ...
        {X_nlp},{horzcat(c_DEL{:})}, {'x_nlp'}, {'cDEL'} ...
        );
    constrDef.Fun_cWS_int  = casadi.Function('cWS_int', ...
        {X_nlp, x_TCP_F},{horzcat(c_WS{:,1})}, ...
        {'x_nlp', 'x_TCP_F'}, {'cWS_int'} ...
        );
    constrDef.Fun_cWS_ext  = casadi.Function('cWS_ext', ...
        {X_nlp, x_TCP_F}, {horzcat(c_WS{:,2})}, ...
        {'x_nlp', 'x_TCP_F'}, {'cWS_ext'} ...
        );

    % Additionally include objective function
    constrDef.Fun_f = casadi.Function('f', ...
        {X_nlp, x_TCP_F, wR, wF}, {J}, {'x_nlp', 'x_TCP_F', 'wR', 'wF'}, ...
        {'J'} ...
        );
    constrDef.Fun_fRComp = casadi.Function('fComponents', ...
        {X_nlp, x_TCP_F, wR}, JR, {'x_nlp', 'x_TCP_F', 'wR'}, ...
        cellstr(arrayfun(@(x) sprintf("JR_%d", x), 1:numel(JR))) ...
        );
    constrDef.Fun_fFComp = casadi.Function('fComponents', ...
        {X_nlp, x_TCP_F, wF}, JF, {'x_nlp', 'x_TCP_F', 'wF'}, ...
        cellstr(arrayfun(@(x) sprintf("JF_%d", x), 1:numel(JF))) ...
        );

    %% Define NLP solver

    fprintf('Defining NLP object... ');
    tic;
    NLprob = struct;
    NLprob.f = J;
    NLprob.x = X_nlp;
    NLprob.g = c;
    NLprob.p = vertcat(wR,wF,x_TCP_F);

    % Define standard/global solver options
    optsGlobal = struct();
    optsGlobal.expand = true;

    % solver: e.g., sqpmethod, ipopt, worhp, scpgen
    solver = 'ipopt';

    % Assemble solver options: Fields from OCP object overwrite standard
    % options
    opts = OCP.nlpOpts;
    fieldsGlobal = fieldnames(optsGlobal);
    for iField = 1:length(fieldsGlobal)
        if ~isfield(opts, fieldsGlobal{iField})
            opts.(fieldsGlobal{iField}) = optsGlobal.(fieldsGlobal{iField});
        end
    end

    NLPSolver = casadi.nlpsol('solver', solver, NLprob, opts);

    tMeas = toc;
    fprintf('took %.3f s.\n', tMeas);
end