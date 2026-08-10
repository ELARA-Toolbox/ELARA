function DEL_res_k = DELResidualInitialStep(system, simPars, q_0, q_1, u_0, qDot0, h, a)
    %% Compute the Residual for the first step in DEL integration
    arguments
        % Multibody system
        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters

        % Generalized coordinates at steps 0 and 1 (nDoF, 1)
        q_0     (:,1)
        q_1     (:,1)

        % Control input at step 0 (nInputs, 1)
        u_0     (:,1)

        % Initial generalized velocity
        qDot0   (:,1)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    %% Forward Kinematics and Velocities
    [g_0, g_rel_0] = system.computeFwdKin(q_0);
    g_rel_1 = system.computeJointTransformations(q_1);

    % Compute absolute velocities
    eta_0 = system.computeDiscreteAbsoluteVelocities(g_rel_0, g_rel_1, h);

    %% Get residual
    DEL_res_k = elara.dynamics.sym.DELResidualInitialStep_noKinematics( ...
        system, simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, qDot0, h, a);
end
