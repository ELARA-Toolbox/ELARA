function DEL_res_k = computeDELResiduumFirstStep_casadi_extKin( ...
        MBSys, simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, qDot0, h, a)
    %% Compute the Residuum for the first step in DEL integration
    % Index convention: Same as in literature; i.e., starting at 0
    arguments
        % Multibody system
        MBSys   (1,1) MBSystemSym

        simPars (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_0     (:,1)
        q_1     (:,1)

        % Externally computed kinematic quantities
        g_0     (:,1) SE3
        g_rel_0 (:,1) SE3
        eta_0   (:,2) cell

        % Vector of inputs at initial time t0 (1,nInputs)
        u_0     (:,1)

        % Vector of initial generalized velocities at t0
        qDot0   (:,1)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    f = getSE3Functions(q_1);

    %% Jacobians, Mass and Input Matrix
    J_0 = MBSys.computeGeomJacobianFast(q_0, g_rel_0);
    M_0 = MBSys.computeMassMatrixFast(J_0);
    B_0 = MBSys.computeInputMatrixFast(g_rel_0);

    %% Forces / EOM Term

    % Generalized forces (stress and dissipation)
    f_gen_0 = (1-a) * MBSys.cSys .* (q_0 - MBSys.qRef) ...
        + (1-a) * MBSys.dSys .* (q_1 - q_0)/h;

    % Placeholder values for external forces
    f_frame_0_b_ext = zeros(6, MBSys.nFrames);
    f_frame_0_s_ext = zeros(6, MBSys.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_0_b = computeBodyfixedFrameForces_sym(MBSys, g_0, f_frame_0_s_ext, simPars.g);


    %% Evaluate DEL / Compute Residual
    DEL_res_0_C = cell(MBSys.nFrames, 1);
    for iFrm = 1:MBSys.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            - h*(1-a)*f_frame_0_b_ext(:, iFrm) ...
            + h*(1-a)*f_frame_0_b{iFrm} ...
            + f.cayRTDInvSE3(eta_0{iFrm,1}*h, eta_0{iFrm,2}*h).' * MBSys.frameData.MGen{iFrm} * vertcat(eta_0{iFrm,:});
        % Distribute node terms to coordinates
        for iB = 1:MBSys.nFrames
            if ~isempty(J_0{iFrm,iB})
                if isempty(DEL_res_0_C{iB})
                    DEL_res_0_C{iB} = J_0{iFrm,iB}.' * f_frame_b_i;
                else
                    DEL_res_0_C{iB} = DEL_res_0_C{iB} + J_0{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    % Add input terms and initial momentum to coordinates
    for iFrm = 1:MBSys.nFrames
        for iInput = 1:MBSys.nInputs
            if ~isempty(B_0{iFrm, iInput})
                DEL_res_0_C{iFrm} = DEL_res_0_C{iFrm} - h*(1-a)*B_0{iFrm, iInput} * u_0(iInput);
            end
        end
        DEL_res_0_C{iFrm} = DEL_res_0_C{iFrm} - horzcat(M_0{iFrm,:}) * qDot0;
    end

    DEL_res_k = vertcat(DEL_res_0_C{:}) + h*f_gen_0;

    % Invert sign to be consistent with the standard form of the DEL equations
    %DEL_res_k = -DEL_res_k;
end
