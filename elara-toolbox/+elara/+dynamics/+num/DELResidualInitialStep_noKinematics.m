function res_0 = DELResidualInitialStep_noKinematics(system, h, simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, qDot0, a)
    %% Compute the Residual for the first step in DEL integration
    % i.e., step 0 -> 1,
    % according to the formula in [Obe08, p.50]
    % based on the Legendre transform to properly include initial velocities
    % Notes:
    %  * We're using the complete trapezoidal rule for the first step
    %    for true second-order accuracy
    arguments
        system  (1,1) elara.abstract.System
        h       (1,1)
        simPars (1,1) elara.SimulationParameters

        % Coordinates at time step k = 1
        q_0     (:,1) double

        % Coordinate variable at time step k = 2
        q_1     (:,1) double

        % Externally computed kinematic quantities
        g_0     (4,4,:) double
        g_rel_0 (4,4,:) double
        eta_0   (6,:) double

        u_0     (:,1) double % System input variable at time step k = 1

        % Initial velocity at t = 0
        qDot0   (:,1) double

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    % Initial (continuous-time) momentum
    J_0 = system.computeGeomJacobianFast(q_0, g_rel_0);
    p_0 = system.computeMassMatrixFast(J_0) * qDot0;

    % Frame forces at first step
    f_frame_0_b_ext = zeros(6, system.nFrames);
    f_frame_0_s_ext = zeros(6, system.nFrames);
    f_frame_0_b = (1-a)*elara.dynamics.num.bodyFixedFrameForces(g_0, f_frame_0_s_ext, system, simPars);

    % Generalized Forces (stresses, actuation and dissipation)
    f_gen_0 = ...
        + (1-a)*system.cSys .* (q_0 - system.qRef) ...
        + (1-a)*system.dSys .* (q_1 - q_0)/h ...
        - (1-a)*system.computeInputMatrixFast(g_rel_0) * u_0;

    % Evaluate DEL
    res_0 = h*f_gen_0 - p_0;
    for iFrm = 1:system.nFrames
        res_0 = res_0 + J_0(:,:,iFrm).' *( ...
            + elara.SE3.dcayInv(eta_0(:,iFrm)*h).' * system.frames.MGen(:,:,iFrm) * eta_0(:,iFrm) ...
            + h*f_frame_0_b(:, iFrm) ...
            - h*(1-a)*f_frame_0_b_ext(:, iFrm) ...
            );
    end
end
