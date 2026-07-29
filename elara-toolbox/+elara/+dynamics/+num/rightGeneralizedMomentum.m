function p_k = rightGeneralizedMomentum(MBSys, h, simPars, q_k, q_k1, g_k1, eta_k, u_k1, a)
    %% Compute the (forced) "Right" Generalized Momentum p_k1
    % based on the forced discrete Legendre transform F+ L_d (q_k, q_k1)
    arguments
        MBSys   (1,1) elara.abstract.System
        h       (1,1)
        simPars (1,1) elara.SimulationParameters

        % Coordinates
        q_k     (:,1) double
        q_k1    (:,1) double

        % Externally computed kinematic quantities
        g_k1    (4,4,:) double
        eta_k   (6,:) double

        % Inputs at time instance t_k1
        u_k1

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    g_rel_k1 = MBSys.computeJointTransformations(q_k1);
    J_k1 = MBSys.computeGeomJacobianFast(q_k1, g_rel_k1);

    % Frame forces
    f_frame_k1_s = zeros(6, MBSys.nFrames);    
    f_frame_k1_b = -h*a*elara.dynamics.num.bodyFixedFrameForces(g_k1, f_frame_k1_s, MBSys, simPars);

    % Generalized Forces (stresses, actuation and dissipation)
    f_gen_k1 = ...
        - h*a*MBSys.cSys .* (q_k1 - MBSys.qRef) ...
        -   a*MBSys.dSys .* (q_k1 - q_k) ...
        + h*a*MBSys.computeInputMatrixFast(g_rel_k1) * u_k1;

    % Compute Momentum
    p_k = f_gen_k1;
    for iFrm = 1:MBSys.nFrames
        p_k = p_k ...
            + J_k1(:,:,iFrm).' *( ...
            + cayRTDInvSE3(eta_k(:,iFrm)*h).' * MBSys.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            + f_frame_k1_b(:, iFrm) ...
            );
    end
end
