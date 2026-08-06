function [res_N, eta_N0] = DELResidualFinalStep(system, h, simPars, q_N0, q_N, u_N, qDotEnd, a)
    %% Compute Residuum for the final step N-1 -> N
    arguments
        system  (1,1) elara.abstract.System
        h       (1,1) 
        simPars (1,1) elara.SimulationParameters
        q_N0    (:,1) % Coordinate variable at time step N
        q_N     (:,1) % Coordinate variable at (final) time step N+1
        u_N     (:,1) % Input variable at time step N

        qDotEnd (:,1) % Final velocity

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1)
    end

    % Forward kinematics for the final two steps (k = N-1, N)
    g_rel_N0 = system.computeJointTransformations(q_N0);
    [g_N, g_rel_N] = system.computeFwdKin(q_N);
    J_N = system.computeGeomJacobianFast(q_N, g_rel_N);

    % Final (continuous-time) momentum
    p_N = system.computeMassMatrixFast(J_N) * qDotEnd;

    % Velocities
    eta_N0 = system.computeDiscreteAbsoluteVelocities(g_rel_N0,  g_rel_N, h);

    % Frame forces at last step
    f_frame_N_b_ext = zeros(6, system.nFrames);
    f_frame_N_s_ext = zeros(6, system.nFrames);
    f_frame_N_b = -a*f_frame_N_b_ext ...
        + a*elara.dynamics.num.bodyFixedFrameForces(g_N, f_frame_N_s_ext, system, simPars);

    % Generalized Forces (stresses, actuation and dissipation)
    f_gen_N = ...
        + a * system.cSys .* (q_N - system.qRef) ...
        + a * system.dSys .* (q_N - q_N0)/h ...
        - a * system.computeInputMatrixFast(g_rel_N) * u_N;
    
    % DEL Eqs. / Legendre transform at last step
    res_N = -h*f_gen_N - p_N;
    for iFrm = 1:system.nFrames
        res_N = res_N + J_N(:,:,iFrm).' * (...
            + elara.SE3.dcayInv(-eta_N0(:,iFrm)*h).' * system.frames.MGen(:,:,iFrm) * eta_N0(:,iFrm) ...
            - h*f_frame_N_b(:, iFrm) ...
            );
    end
end
