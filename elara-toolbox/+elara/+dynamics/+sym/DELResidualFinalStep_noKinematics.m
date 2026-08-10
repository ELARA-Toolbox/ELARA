function DEL_res_N = DELResidualFinalStep_noKinematics( ...
        system, simPars, q_N0, q_N, g_N, g_rel_N, eta_N0, u_N, ...
        qDotF, h, a)
    %% Compute Residual of DEL Equation
    % Index convention: Same as in literature; i.e., starting at 0
    % (i.e., last index is N)
    arguments
        % Multibody system
        system   (1,1) elara.SystemSym

        simPars  (1,1) elara.SimulationParameters

        % Generalized coordinates at steps N-1 and N (nDoF, 1)
        q_N0     (:,1)
        q_N      (:,1)

        % Externally computed kinematic quantities
        g_N      (:,1) elara.SE3.Element
        g_rel_N  (:,1) elara.SE3.Element
        eta_N0   (:,2) cell
 
        % Control input at the final time (nInputs, 1)
        u_N      (:,1)

        % Generalized velocity at the end
        qDotF  (:,1)

        % Time step
        h        (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a        (1,1)
    end

    f = elara.internal.math.getSE3Functions(q_N);


    %% Jacobians, Mass and Input Matrix
    J_N = system.computeGeomJacobianFast(q_N, g_rel_N);
    M_N = system.computeMassMatrixFast(J_N);
    B_N = system.computeInputMatrixFast(g_rel_N);

    %% Forces / EOM Term

    % Generalized forces (stress and dissipation)
    f_gen_N = ...
        + a * system.cSys .* (q_N - system.qRef) ...
        + a * system.dSys .* (q_N - q_N0)/h;

    % Placeholder values for external forces
    f_frame_N_b_ext = zeros(6, system.nFrames);
    f_frame_N_s_ext = zeros(6, system.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_k_b = elara.dynamics.sym.bodyFixedFrameForces(system, g_N, f_frame_N_s_ext, simPars.g);


    %% Evaluate DEL / Compute Residual
    DEL_res_k_C = cell(system.nFrames, 1);
    for iFrm = 1:system.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            + h*a*f_frame_N_b_ext(:, iFrm) ...
            - h*a*f_frame_k_b{iFrm} ...
            + f.SE3.dcayInv(-eta_N0{iFrm,1}*h, -eta_N0{iFrm,2}*h).' * system.frames.MGen{iFrm} * vertcat(eta_N0{iFrm,:});

        % Distribute frame terms to coordinates
        for iB = 1:system.nFrames
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
    for iFrm = 1:system.nFrames
        for iInput = 1:system.nInputs
            if ~isempty(B_N{iFrm, iInput})
                DEL_res_k_C{iFrm} = DEL_res_k_C{iFrm} + h*a*B_N{iFrm, iInput} * u_N(iInput);
            end
        end
        DEL_res_k_C{iFrm} = DEL_res_k_C{iFrm} - horzcat(M_N{iFrm,:}) * qDotF;
    end

    DEL_res_N = vertcat(DEL_res_k_C{:}) - h*f_gen_N;

end
