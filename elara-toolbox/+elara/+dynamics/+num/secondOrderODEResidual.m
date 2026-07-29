function f = secondOrderODEResidual(t, q, q_dot, q_ddot, u, MBSys, simPars) %#codegen
    %% Compute the full second-order ODE
    arguments (Input)
        % Integration time (from ode solver)
        t           (1,1) double

        % Configuration vector and its derivatives (nDof,1)
        q           (:,1) double
        q_dot       (:,1) double
        q_ddot      (:,1) double

        % Input vector
        u           (:,1) double

        MBSys       (1,1) elara.abstract.System

        simPars     (1,1) elara.SimulationParameters
    end
    arguments (Output)
        f        (:,1) double
    end

    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = MBSys.computeFwdKin(q);
    J = MBSys.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    eta = zeros(6,MBSys.nFrames);
    for iFrm = 1:MBSys.nFrames
        eta(:,iFrm) = J(:,:,iFrm) * q_dot;
    end
    J_dot = MBSys.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);


    %% Evaluate EOM

    % Generalized forces (stress, dissipation and system inputs)
    f_gen = MBSys.cSys .* (q - MBSys.qRef) ...
        + MBSys.dSys .* q_dot ...
        - MBSys.computeInputMatrix(q) * u;

    % External frame forces from the environment
    f_frame_b = simPars.externalWrench_b.getCurrentWrench(MBSys.nFrames, t);
    f_frame_s = simPars.externalWrench_s.getCurrentWrench(MBSys.nFrames, t);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + elara.dynamics.num.bodyFixedFrameForces(g, f_frame_s, MBSys, simPars);

    % Compute sum of generalized forces
    for iFrm = 1:MBSys.nFrames
        f_gen = f_gen + J(:,:,iFrm).' * (...
            ...% Frame forces
            + f_frame_b(:,iFrm)...
            ...% Coriolis Term
            + (MBSys.frames.MGen(:,:,iFrm) * J_dot(:,:,iFrm) ...
            - elara.SE3.smallAd(eta(:,iFrm)).' * MBSys.frames.MGen(:,:,iFrm) * J(:,:,iFrm)) * q_dot ...
            );
    end

    %% Assemble full second-order ODE
    M = MBSys.computeMassMatrix(q);
    f = -(M*q_ddot + f_gen );

end
