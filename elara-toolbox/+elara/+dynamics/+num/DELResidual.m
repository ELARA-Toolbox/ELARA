function DEL_res_k = DELResidual(MBSys, simPars, q_k0, q_k, q_k1, u_k, h, a)
    %% Compute Residuum of DEL Equation
    % only in terms of coordinates q (at time steps k-1, k, k+1)
    arguments
        % Multibody system
        MBSys   (1,1) elara.SystemNum

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
    [g_k, g_rel_k]  = MBSys.computeFwdKin(q_k);
    [~,   g_rel_k0] = MBSys.computeFwdKin(q_k0);
    [~,   g_rel_k1] = MBSys.computeFwdKin(q_k1);

    %% Compute absolute velocities
    eta_k0 = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k0, g_rel_k, h);
    eta_k  = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);

    %% Evaluate DEL / Compute Residual

    % Placeholder values for external forces
    f_frame_k_b_ext = zeros(6, MBSys.nFrames);
    f_frame_k_s_ext = zeros(6, MBSys.nFrames);

    DEL_res_k = elara.dynamics.num.DELResidual_noKinematics(MBSys, simPars, ...
        q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, u_k, ...
        f_frame_k_b_ext, f_frame_k_s_ext, h, a);
end
