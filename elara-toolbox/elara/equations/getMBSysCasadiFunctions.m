function funs = getMBSysCasadiFunctions(MBSys, h, simPars, MGenCell)
    %% Get Casadi functions for MBSys object
    arguments
        MBSys   (1,1) MBSystemSym
        h       (1,1) casadi.DM
        simPars (1,1) MBSimPars

        % Cell array of generalized mass matrices
        % Specified as an extra argument to use generalized mass matrices
        % with symbolic variables (for parameter identification)
        MGenCell (:,1) cell = {}
    end

    % Get generalized mass matrix array from numeric array if not given
    if isempty(MGenCell)
        MGenCell = squeeze(num2cell(MBSys.frameData.MGen,[1,2]));
    end

    funOpts = struct();
    funOpts.cse = true; % Common subexpression elimination

    funs = struct();

    %% Variables
    %h = casadi.SX.sym('h', 1, 1);
    q_k  = casadi.MX.sym('q_k', MBSys.nDoF, 1);
    q_k1 = casadi.MX.sym('q_k1', MBSys.nDoF, 1);
    u_k  = casadi.MX.sym('u_k',  MBSys.nInputs, 1);

    f = getSE3Functions(q_k);


    %% Relative joint transformations
    g_rel_k = MBSys.computeJointTransformations(q_k);
    argNames_g_rel_RxCell = cellstr([ ...
        arrayfun(@(x) sprintf("R_rel_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("x_rel_%d", x), 1:MBSys.nFrames) ...
        ]);

    funs.computeJointTransformations = casadi.Function('jointTransformations', ...
        {q_k}, [{g_rel_k.R},{g_rel_k.x}], ...
        {'q_k'}, argNames_g_rel_RxCell, ...
        funOpts);


    %% Forward kinematics

    % Create symbolic input arguments
    sym_g_rel_SE3 = createArray(MBSys.nFrames, 1, "SE3");
    sym_g_rel_RxCell = cell(MBSys.nFrames,2);
    for iFrm = 1:MBSys.nFrames
        R = casadi.MX.sym(sprintf('R_rel_%d', iFrm),3,3);
        x = casadi.MX.sym(sprintf('x_rel_%d', iFrm),3,1);
        sym_g_rel_SE3(iFrm).R = R;
        sym_g_rel_SE3(iFrm).x = x;
        sym_g_rel_RxCell{iFrm, 1} = R;
        sym_g_rel_RxCell{iFrm, 2} = x;
    end

    g_k = MBSys.computeFwdKinFast(sym_g_rel_SE3);
    argNames_g_RxCell = cellstr([ ...
        arrayfun(@(x) sprintf("R_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("x_%d", x), 1:MBSys.nFrames) ...
        ]);

    funs.computeFwdKinFast = casadi.Function('FwdKinFast', ...
        sym_g_rel_RxCell(:), [{g_k.R},{g_k.x}], ...
        argNames_g_rel_RxCell, argNames_g_RxCell, ...
        funOpts);


    %% Discrete Absolute Velocities
    sym_g_rel_k_SE3  = createArray(MBSys.nFrames, 1, "SE3");
    sym_g_rel_k1_SE3 = createArray(MBSys.nFrames, 1, "SE3");
    sym_g_rel_k_RxCell  = cell(MBSys.nFrames,2);
    sym_g_rel_k1_RxCell = cell(MBSys.nFrames,2);

    for iFrm = 1:MBSys.nFrames
        R_k = casadi.MX.sym(sprintf('R_rel_k_%d', iFrm),3,3);
        x_k = casadi.MX.sym(sprintf('x_rel_k_%d', iFrm),3,1);
        sym_g_rel_k_SE3(iFrm).R = R_k;
        sym_g_rel_k_SE3(iFrm).x = x_k;
        sym_g_rel_k_RxCell{iFrm, 1} = R_k;
        sym_g_rel_k_RxCell{iFrm, 2} = x_k;

        R_k1 = casadi.MX.sym(sprintf('R_rel_k1_%d', iFrm),3,3);
        x_k1 = casadi.MX.sym(sprintf('x_rel_k1_%d', iFrm),3,1);
        sym_g_rel_k1_SE3(iFrm).R = R_k1;
        sym_g_rel_k1_SE3(iFrm).x = x_k1;
        sym_g_rel_k1_RxCell{iFrm, 1} = R_k1;
        sym_g_rel_k1_RxCell{iFrm, 2} = x_k1;
    end

    argNames_g_rel_k_RxCell = cellstr([ ...
        arrayfun(@(x) sprintf("R_rel_k_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("x_rel_k_%d", x), 1:MBSys.nFrames) ...
        ]);
    argNames_g_rel_k1_RxCell = cellstr([ ...
        arrayfun(@(x) sprintf("R_rel_k1_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("x_rel_k1_%d", x), 1:MBSys.nFrames) ...
        ]);
    argNames_eta_k_omVCell = cellstr([ ...
        arrayfun(@(x) sprintf("omega_k_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("v_k_%d", x), 1:MBSys.nFrames) ...
        ]);

    eta_k = MBSys.computeDiscreteAbsoluteVelocities( ...
        sym_g_rel_k_SE3, sym_g_rel_k1_SE3, h);

    funs.computeDiscreteAbsoluteVelocities = casadi.Function('discAbsoluteVelocities', ...
        [sym_g_rel_k_RxCell(:); sym_g_rel_k1_RxCell(:)], eta_k(:), ...
        [argNames_g_rel_k_RxCell, argNames_g_rel_k1_RxCell], argNames_eta_k_omVCell, ...
        funOpts);


    %% Input term
    B_k = MBSys.computeInputMatrixFast(sym_g_rel_SE3);
    B_term = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        for iInput = 1:MBSys.nInputs
            if ~isempty(B_k{iFrm, iInput})
                if isempty(B_term{iFrm})
                    B_term{iFrm} = B_k{iFrm, iInput} * u_k(iInput);
                else
                    B_term{iFrm} = B_term{iFrm} + B_k{iFrm, iInput} * u_k(iInput);
                end
            end
        end
    end

    argNames_inputTermCell = cellstr(arrayfun(@(x) sprintf("Bu_%d", x), 1:MBSys.nFrames));

    funs.inputTerm = casadi.Function('InputTerm', ...
        [{u_k}; sym_g_rel_RxCell(:)], B_term, ...
        [{'u'}, argNames_g_rel_RxCell], argNames_inputTermCell, ...
        funOpts);


    %% Generalized forces: Stiffness and dissipation

    f_gen_c_k = casadi.DM(MBSys.cSys) .* (q_k - MBSys.qRef);
    f_gen_d_k = casadi.DM(MBSys.dSys) .* (q_k1-q_k)/h;
    f_gen_d_k_diff = casadi.DM(MBSys.dSys) .* (q_k1-q_k);
    funs.f_gen_c = casadi.Function('f_gen_c', {q_k}, {f_gen_c_k}, {'q'}, {'f_gen_c'}, funOpts);
    funs.f_gen_d = casadi.Function('f_gen_d', {q_k, q_k1}, {f_gen_d_k}, {'q_k', 'q_k1'}, {'f_gen_d'}, funOpts);
    funs.f_gen_d_diff = casadi.Function('f_gen_d_diff', {q_k, q_k1}, {f_gen_d_k_diff}, {'q_k', 'q_k1'}, {'f_gen_d_diff'}, funOpts);


    %% Frame forces: Ext. forces and gravity

    f_frame_s_k = casadi.MX.sym('f_frame_s_k', 6, MBSys.nFrames);

    sym_g_SE3 = createArray(MBSys.nFrames, 1, "SE3");
    sym_g_RxCell = cell(MBSys.nFrames,2);
    for iFrm = 1:MBSys.nFrames
        R = casadi.MX.sym(sprintf('R_%d', iFrm),3,3);
        x = casadi.MX.sym(sprintf('x_%d', iFrm),3,1);
        sym_g_SE3(iFrm).R = R;
        sym_g_SE3(iFrm).x = x;
        sym_g_RxCell{iFrm, 1} = R;
        sym_g_RxCell{iFrm, 2} = x;
    end
    argNames_g_RxCell = cellstr([ ...
        arrayfun(@(x) sprintf("R_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("x_%d", x), 1:MBSys.nFrames) ...
        ]);
    argNames_bfFFCell = cellstr(arrayfun(@(x) sprintf("bfFF_%d", x), 1:MBSys.nFrames));

    bodyFixedFrameForces = computeBodyfixedFrameForces_sym( ...
        MBSys, sym_g_SE3, f_frame_s_k, simPars.g);

    funs.bodyFixedFrameForces = casadi.Function('bodyFixedFrameForces', ...
        [{f_frame_s_k}; sym_g_RxCell(:)], bodyFixedFrameForces, ...
        [{'f_frame_s_k'}, argNames_g_RxCell], argNames_bfFFCell, ...
        funOpts);


    %% Frame forces: Inertia term with mu_k0
    sym_etaCell = cell(MBSys.nFrames,2);
    for iFrm = 1:MBSys.nFrames
        sym_etaCell{iFrm, 1} = casadi.MX.sym(sprintf('om_%d', iFrm),3,1);
        sym_etaCell{iFrm, 2} = casadi.MX.sym(sprintf('v_%d', iFrm),3,1);
    end
    argNames_eta_k0_cell = cellstr([ ...
        arrayfun(@(x) sprintf("om_k0_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("v_k0_%d", x), 1:MBSys.nFrames) ...
        ]);
    argNames_FITCell = cellstr(arrayfun(@(x) sprintf("FIT_%d", x), 1:MBSys.nFrames));

    frameInertiaTerm = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        frameInertiaTerm{iFrm} = f.cayRTDInvSE3(-sym_etaCell{iFrm,1}*h, -sym_etaCell{iFrm,2}*h).' * MGenCell{iFrm} * vertcat(sym_etaCell{iFrm,:});
    end

    funs.frameInertiaTerm = casadi.Function('frameInertiaTerm', ...
        sym_etaCell(:), frameInertiaTerm, ...
        argNames_eta_k0_cell, argNames_FITCell, ...
        funOpts);


    %% Frame forces: Discrete momentum mu_k

    argNames_eta_k_cell = cellstr([ ...
        arrayfun(@(x) sprintf("om_k_%d", x), 1:MBSys.nFrames), ...
        arrayfun(@(x) sprintf("v_k_%d", x), 1:MBSys.nFrames) ...
        ]);
    argNames_mu_k_cell = cellstr(arrayfun(@(x) sprintf("mu_k_%d", x), 1:MBSys.nFrames));

    mu_k = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        mu_k{iFrm} = f.cayRTDInvSE3(sym_etaCell{iFrm,1}*h, sym_etaCell{iFrm,2}*h).' * MGenCell{iFrm} * vertcat(sym_etaCell{iFrm,:});
    end

    funs.discreteMomentum = casadi.Function('discreteMomentum', ...
        sym_etaCell(:), mu_k, ...
        argNames_eta_k_cell, argNames_mu_k_cell, ...
        funOpts);


    %% Distribution of frame forces to generalized forces

    sym_f_frame_cell = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        sym_f_frame_cell{iFrm} = casadi.MX.sym(sprintf('f_frame_%d', iFrm),6,1);
    end
    argNames_f_frame_cell = cellstr(arrayfun(@(x) sprintf("f_frame_%d", x), 1:MBSys.nFrames));
    argNames_f_gen_cell = cellstr(arrayfun(@(x) sprintf("f_gen_%d", x), 1:MBSys.nFrames));

    J_k = MBSys.computeGeomJacobianFast(q_k, sym_g_rel_k_SE3);
    f_gen_k_cell = cell(MBSys.nFrames,1);

    for iFrm = 1:MBSys.nFrames
        for iB = 1:MBSys.nFrames
            if ~isempty(J_k{iFrm,iB})
                if isempty(f_gen_k_cell{iB})
                    f_gen_k_cell{iB} = J_k{iFrm,iB}.' * sym_f_frame_cell{iFrm};
                else
                    f_gen_k_cell{iB} = f_gen_k_cell{iB} + J_k{iFrm,iB}.' * sym_f_frame_cell{iFrm};
                end
            end
        end
    end

    funs.frameForcesToGenForces = casadi.Function('frameForcesToGenForces', ...
        [{q_k}; sym_g_rel_k_RxCell(:); sym_f_frame_cell], f_gen_k_cell, ...
        [{'q_k'}, argNames_g_rel_RxCell, argNames_f_frame_cell], argNames_f_gen_cell, ...
        funOpts);


    %% Mass matrix
    M = MBSys.computeMassMatrix(q_k);
    funs.computeMassMatrix = casadi.Function('massMatrix', ...
        {q_k}, M(:), ...
        funOpts);

end
