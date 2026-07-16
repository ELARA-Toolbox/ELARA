function DEL_res_k = computeDELResiduum_extKin( MBSys, simPars, ...
        q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, u_k, ...
        f_frame_k_b_ext, f_frame_k_s_ext, h, a)
    %% Compute Residuum of DEL Equation
    arguments
        % Multibody system
        MBSys   (1,1) elara.SystemNum

        simPars (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_k0    (:,1) double
        q_k     (:,1) double
        q_k1    (:,1) double

        % Externally computed kinematic quantities
        g_k     (4,4,:) double
        g_rel_k (4,4,:) double
        eta_k   (6,:) double
        eta_k0  (6,:) double

        % Vector of inputs (1,nInputs)
        u_k     (:,1) double

        % Matrix of external wrenches
        f_frame_k_b_ext (6,:) double
        f_frame_k_s_ext (6,:) double

        % Time step
        h       (1,1) double

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    % Check argument sizes
    assert( size(q_k,1) == MBSys.nDoF, ...
        "Vector of generalized coordinates has wrong dimensions.");
    assert( size(u_k,1) == MBSys.nInputs, ...
        "Vector of inputs has wrong dimensions.");

    %% Jacobians
    J_k = MBSys.computeGeomJacobianFast(q_k, g_rel_k);

    %% Forces / EOM Terms

    % Generalized forces (stress, dissipation and actuation)
    f_gen_k = MBSys.cSys .* (q_k - MBSys.qRef) ...
        + (1-a)/h * MBSys.dSys .* (q_k1-q_k) ...
        + a/h* MBSys.dSys .* (q_k-q_k0)...
        - MBSys.computeInputMatrix(q_k) * u_k;

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_k_b = computeBodyfixedFrameForces(g_k, f_frame_k_s_ext, MBSys, simPars);

    %% Evaluate DEL / Compute Residual
    DEL_res_k = f_gen_k;
    for iFrm = 1:MBSys.nFrames
        DEL_res_k = DEL_res_k ...
            + J_k(:,:,iFrm).' *( ...
            1/h*cayRTDInvSE3(eta_k(:,iFrm)*h).' * MBSys.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            - 1/h*cayRTDInvSE3(-eta_k0(:,iFrm)*h).' * MBSys.frames.MGen(:,:,iFrm) * eta_k0(:,iFrm) ...
            + f_frame_k_b(:, iFrm) ...
            - f_frame_k_b_ext(:, iFrm) ...
            ... % Quadratic dissipation in absolute velocities
            ... %+ h* discPars.dQuad .* eta_k(:,iN-1).^2 .* sign(eta_k(:,iN-1)) ...
            );
    end

    % Invert sign to be consistent with the standard form of the DEL equations
    DEL_res_k = -DEL_res_k;
end