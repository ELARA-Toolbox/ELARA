function p_k = leftGeneralizedMomentum(system, h, simPars, q_k, q_k1, g_k, eta_k, u_k, a)
    %% Compute the (forced) "Left" Generalized Momentum p_k
    % based on the forced discrete Legendre transform F- L_d (q_k, q_k1)
    arguments
        system  (1,1) elara.abstract.System
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

    g_rel_k = system.computeJointTransformations(q_k);
    J_k = system.computeGeomJacobianFast(q_k, g_rel_k);

    % Frame forces
    f_frame_k_s = zeros(6, system.nFrames);    
    f_frame_k_b = h*(1-a)*elara.dynamics.num.bodyFixedFrameForces(g_k, f_frame_k_s, system, simPars);

    % Generalized Forces (stresses, actuation and dissipation)
    f_gen_k = ...
        + h*(1-a)*system.cSys .* (q_k - system.qRef) ...
        +   (1-a)*system.dSys .* (q_k1-q_k) ...
        - h*(1-a)*system.computeInputMatrixFast(g_rel_k) * u_k;

    % Compute Momentum
    p_k = f_gen_k;
    for iFrm = 1:system.nFrames
        p_k = p_k + J_k(:,:,iFrm).' *( ...
            + elara.SE3.dcayInv(eta_k(:,iFrm)*h).' * system.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            + f_frame_k_b(:, iFrm) ...
            );
    end
end
