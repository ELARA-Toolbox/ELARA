function DEL_res_k = DELResidual(system, simPars, q_k0, q_k, q_k1, u_k, h, a)
    %% Compute Residual of DEL Equation
    % only in terms of coordinates q (at time steps k-1, k, k+1)
    arguments
        % Multibody system
        system  (1,1) elara.SystemNum

        simPars (1,1) elara.SimulationParameters

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_k0    (:,1) double
        q_k     (:,1) double
        q_k1    (:,1) double

        % Vector of inputs (1,nInputs)
        u_k     (:,1) double

        % Time step
        h       (1,1) double

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    %% Forward Kinematics
    [g_k, g_rel_k] = system.computeFwdKin(q_k);
    g_rel_k0 = system.computeJointTransformations(q_k0);
    g_rel_k1 = system.computeJointTransformations(q_k1);

    %% Compute absolute velocities
    eta_k0 = system.computeDiscreteAbsoluteVelocities(g_rel_k0, g_rel_k, h);
    eta_k  = system.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);

    %% Evaluate DEL / Compute Residual

    % Placeholder values for external forces
    f_frame_k_b_ext = zeros(6, system.nFrames);
    f_frame_k_s_ext = zeros(6, system.nFrames);

    DEL_res_k = elara.dynamics.num.DELResidual_noKinematics(system, simPars, ...
        q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, u_k, ...
        f_frame_k_b_ext, f_frame_k_s_ext, h, a);
end
