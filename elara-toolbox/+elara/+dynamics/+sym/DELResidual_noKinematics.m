function DEL_res_k = DELResidual_noKinematics(system, simPars, ...
        q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, ...
        u_k, f_frame_k_b_ext, f_frame_k_s_ext, h, a)
    %% Compute Residual of DEL Equation
    arguments
        % Multibody system
        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters

        % Generalized coordinates at steps k-1, k, and k+1 (nDoF, 1)
        q_k0    (:,1)
        q_k     (:,1)
        q_k1    (:,1)

        % Externally computed kinematic quantities
        g_k     (:,1) elara.SE3.Element
        g_rel_k (:,1) elara.SE3.Element
        eta_k   (:,2) cell
        eta_k0  (:,2) cell

        % Control input at step k (nInputs, 1)
        u_k     (:,1)

        % Matrix of external wrenches
        % TODO: Implement "cleanly" as cell arrays (like twists)
        f_frame_k_b_ext (6,:)
        f_frame_k_s_ext (6,:)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    f = elara.internal.math.getSE3Functions(q_k);

    %% Forces / EOM Term

    % Jacobians
    J_k = system.computeGeomJacobianFast(q_k, g_rel_k);

    % Input matrix
    B_k = system.computeInputMatrixFast(g_rel_k);

    % Generalized forces (stress and dissipation)
    f_gen_k =  system.cSys .* (q_k - system.qRef) ...
        + (1-a) * system.dSys .* (q_k1 - q_k)/h ...
        + a *     system.dSys .* (q_k - q_k0)/h;

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_k_b = elara.dynamics.sym.bodyFixedFrameForces(system, g_k, f_frame_k_s_ext, simPars.g);


    %% Evaluate DEL / Compute Residual
    DEL_res_k_C = cell(system.nFrames, 1);
    for iFrm = 1:system.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            - h*f_frame_k_b_ext(:, iFrm) ...
            + h*f_frame_k_b{iFrm} ...
            + f.SE3.dcayInv(eta_k{iFrm,1}*h, eta_k{iFrm,2}*h).' * system.frames.MGen{iFrm} * vertcat(eta_k{iFrm,:}) ...
            - f.SE3.dcayInv(-eta_k0{iFrm,1}*h, -eta_k0{iFrm,2}*h).' * system.frames.MGen{iFrm} * vertcat(eta_k0{iFrm,:});

        % Distribute node terms to coordinates
        for iB = 1:system.nFrames
            if ~isempty(J_k{iFrm,iB})
                if isempty(DEL_res_k_C{iB})
                    DEL_res_k_C{iB} = J_k{iFrm,iB}.' * f_frame_b_i;
                else
                    DEL_res_k_C{iB} = DEL_res_k_C{iB} + J_k{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    % Add input terms to coordinates
    for iFrm = 1:system.nFrames
        for iInput = 1:system.nInputs
            if ~isempty(B_k{iFrm, iInput})
                DEL_res_k_C{iFrm} = DEL_res_k_C{iFrm} - h*B_k{iFrm, iInput} * u_k(iInput);
            end
        end
    end

    DEL_res_k = vertcat(DEL_res_k_C{:}) + h*f_gen_k;

    % Invert sign to be consistent with the standard form of the DEL equations
    DEL_res_k = -DEL_res_k;
end
