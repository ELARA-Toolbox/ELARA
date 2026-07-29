function p_k = leftGeneralizedMomentum(MBSys, h, simPars, q_k, q_k1, g_k, eta_k, u_k, a)
    %% Compute the (forced) "Left" Generalized Momentum p_k
    % based on the forced discrete Legendre transform F- L_d (q_k, q_k1)
    arguments
        MBSys   (1,1) elara.abstract.System
        h       (1,1)
        simPars (1,1) elara.SimulationParameters

        % Coordinates at times t_k and t_k1
        q_k     (:,1) double
        q_k1    (:,1) double

        % Externally computed kinematic quantities
        g_k     (4,4,:) double
        eta_k   (6,:) double

        % Inputs at time t_k
        u_k     (:,1) double

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    g_rel_k = MBSys.computeJointTransformations(q_k);
    J_k = MBSys.computeGeomJacobianFast(q_k, g_rel_k);

    % Frame forces
    f_frame_k_s = zeros(6, MBSys.nFrames);    
    f_frame_k_b = h*(1-a)*elara.dynamics.num.bodyFixedFrameForces(g_k, f_frame_k_s, MBSys, simPars);

    % Generalized Forces (stresses, actuation and dissipation)
    f_gen_k = ...
        + h*(1-a)*MBSys.cSys .* (q_k - MBSys.qRef) ...
        +   (1-a)*MBSys.dSys .* (q_k1-q_k) ...
        - h*(1-a)*MBSys.computeInputMatrixFast(g_rel_k) * u_k;

    % Compute Momentum
    p_k = f_gen_k;
    for iFrm = 1:MBSys.nFrames
        p_k = p_k + J_k(:,:,iFrm).' *( ...
            + cayRTDInvSE3(eta_k(:,iFrm)*h).' * MBSys.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            + f_frame_k_b(:, iFrm) ...
            );
    end
end
