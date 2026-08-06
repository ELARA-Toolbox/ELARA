function DEL_res_N = DELResidualFinalStep( ...
        system, simPars, q_N0, q_N, u_N, qDotEnd, h, a)
    %% Compute Residuum of DEL Equation
    arguments
        % Multibody system
        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters

        % Vector of generalized coordinates (1,nDoF) at steps k0, k and k+1
        q_N0    (:,1)
        q_N     (:,1)

        % Vector of inputs (1,nInputs)
        u_N       (:,1)

        % Generalized velocity at the end
        qDotEnd   (:,1)

        % Time step
        h       (1,1)

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    %% Forward Kinematics and Jacobians
    [g_N, g_rel_N] = system.computeFwdKin(q_N);
    g_rel_N0 = system.computeJointTransformations(q_N0);

    % Compute absolute velocities
    eta_N0 = system.computeDiscreteAbsoluteVelocities(g_rel_N0, g_rel_N, h);

    %% Get residual
    DEL_res_N = elara.dynamics.sym.DELResidualFinalStep_noKinematics( ...
        system, simPars, q_N0, q_N, g_N, g_rel_N, eta_N0, u_N, ...
        qDotEnd, h, a);
end
