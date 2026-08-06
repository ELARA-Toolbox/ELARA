function DEL_res_k = DELResidual(system, simPars, q_k0, q_k, q_k1, u_k, h, a)
    %% Compute Residuum of DEL Equation
    arguments
        % Multibody system
        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_k0    (:,1)
        q_k     (:,1)
        q_k1    (:,1)

        % Vector of inputs (1,nInputs)
        u_k       (:,1)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    % Check argument sizes
    assert( size(q_k,1) == system.nDoF, ...
        "Vector of generalized coordinates has wrong dimensions.");
    assert( size(u_k,1) == system.nInputs, ...
        "Vector of inputs has wrong dimensions.");

    %% Forward Kinematics and Jacobians
    [g_k, g_rel_k]  = system.computeFwdKin(q_k);
    g_rel_k0 = system.computeJointTransformations(q_k0);
    g_rel_k1 = system.computeJointTransformations(q_k1);

    eta_k0 = system.computeDiscreteAbsoluteVelocities(g_rel_k0, g_rel_k, h);
    eta_k  = system.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);

    %% External frame forces from the environment
    % % NOTE: May be replaced with symbolic variables or similar.
    %f_frame_k_b = simPars.externalWrench_b.getCurrentWrench(MBSys.nFrames, t);
    %f_frame_k_s = simPars.externalWrench_s.getCurrentWrench(MBSys.nFrames, t);
    f_frame_k_b_ext = zeros(6, system.nFrames);
    f_frame_k_s_ext = zeros(6, system.nFrames);


    %% Get Residual
    DEL_res_k = elara.dynamics.sym.DELResidual_noKinematics(system, simPars, ...
        q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, ...
        u_k, f_frame_k_b_ext, f_frame_k_s_ext, h, a);
end
