function DEL_res_k = computeDELResiduumFirstStep_casadi(MBSys, simPars, q_0, q_1, u_0, qDot0, h, a)
    %% Compute the Residuum for the first step in DEL integration
    arguments
        % Multibody system
        MBSys   (1,1) MBSystemSym

        simPars (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_0     (:,1)
        q_1     (:,1)

        % Vector of inputs (1,nInputs)
        u_0     (:,1)

        % Initial generalized velocity
        qDot0   (:,1)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    %% Forward Kinematics and Velocities
    [g_0, g_rel_0]  = MBSys.computeFwdKin(q_0);
    [~,   g_rel_1] = MBSys.computeFwdKin(q_1);

    % Compute absolute velocities
    eta_0 = MBSys.computeDiscreteAbsoluteVelocities(g_rel_0, g_rel_1, h);

    %% Get residual
    DEL_res_k = computeDELResiduumFirstStep_casadi_extKin(MBSys, simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, qDot0, h, a);
end
