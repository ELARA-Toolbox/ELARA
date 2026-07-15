function DEL_res_N = computeDELResiduumFinalStep_casadi_extKin( ...
        MBSys, simPars, q_N0, q_N, g_N, g_rel_N, eta_N0, u_N, ...
        qDotF, h, a)
    %% Compute Residuum of DEL Equation
    % Index convention: Same as in literature; i.e., starting at 0
    % (i.e., last index is N)
    arguments
        % Multibody system
        MBSys    (1,1) elara.SystemSym

        simPars  (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF) at steps N and N+1
        q_N0     (:,1)
        q_N      (:,1)

        % Externally computed kinematic quantities
        g_N      (:,1) SE3
        g_rel_N  (:,1) SE3
        eta_N0   (:,2) cell
 
        % Vector of inputs at final time tF (1,nInputs)
        u_N      (:,1)

        % Generalized velocity at the end
        qDotF  (:,1)

        % Time step
        h        (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a        (1,1)
    end

    f = getSE3Functions(q_N);


    %% Jacobians, Mass and Input Matrix
    J_N = MBSys.computeGeomJacobianFast(q_N, g_rel_N);
    M_N = MBSys.computeMassMatrixFast(J_N);
    B_N = MBSys.computeInputMatrixFast(g_rel_N);

    %% Forces / EOM Term

    % Generalized forces (stress and dissipation)
    f_gen_N = ...
        + a * MBSys.cSys .* (q_N - MBSys.qRef) ...
        + a * MBSys.dSys .* (q_N - q_N0)/h;

    % Placeholder values for external forces
    f_frame_N_b_ext = zeros(6, MBSys.nFrames);
    f_frame_N_s_ext = zeros(6, MBSys.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_k_b = computeBodyfixedFrameForces_sym(MBSys, g_N, f_frame_N_s_ext, simPars.g);


    %% Evaluate DEL / Compute Residual
    DEL_res_k_C = cell(MBSys.nFrames, 1);
    for iFrm = 1:MBSys.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            + h*a*f_frame_N_b_ext(:, iFrm) ...
            - h*a*f_frame_k_b{iFrm} ...
            + f.cayRTDInvSE3(-eta_N0{iFrm,1}*h, -eta_N0{iFrm,2}*h).' * MBSys.frameData.MGen{iFrm} * vertcat(eta_N0{iFrm,:});

        % Distribute frame terms to coordinates
        for iB = 1:MBSys.nFrames
            if ~isempty(J_N{iFrm,iB})
                if isempty(DEL_res_k_C{iB})
                    DEL_res_k_C{iB} = J_N{iFrm,iB}.' * f_frame_b_i;
                else
                    DEL_res_k_C{iB} = DEL_res_k_C{iB} + J_N{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    % Add input terms and final momentum to coordinates
    for iFrm = 1:MBSys.nFrames
        for iInput = 1:MBSys.nInputs
            if ~isempty(B_N{iFrm, iInput})
                DEL_res_k_C{iFrm} = DEL_res_k_C{iFrm} + h*a*B_N{iFrm, iInput} * u_N(iInput);
            end
        end
        DEL_res_k_C{iFrm} = DEL_res_k_C{iFrm} - horzcat(M_N{iFrm,:}) * qDotF;
    end

    DEL_res_N = vertcat(DEL_res_k_C{:}) - h*f_gen_N;

end
