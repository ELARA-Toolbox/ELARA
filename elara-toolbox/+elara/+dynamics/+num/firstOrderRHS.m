function f_fo = firstOrderRHS(t, x, system, simPars) %#codegen
    %% Compute the Right-Hand Side of the EOMs in first-order form
    arguments (Input)
        % Integration time (from ode solver)
        t           (1,1) double

        % State vector x = [q; q_dot] (2*nDoF, 1)
        x           (:,1) double

        system      (1,1) elara.SystemNum

        simPars     (1,1) elara.SimulationParameters
    end
    arguments (Output)
        f_fo        (:,1) double
    end

    %% Get configuration and velocity
    q     = x(1:system.nDoF);
    q_dot = x((system.nDoF+1):end);


    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = system.computeFwdKin(q);
    J = system.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    eta = zeros(6,system.nFrames);
    for iFrm = 1:system.nFrames
        eta(:,iFrm) = J(:,:,iFrm) * q_dot;
    end
    J_dot = system.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);

    %% Prepare system inputs
    % Constant inputs
    if ~isempty(simPars.uConst) && size(simPars.uConst,1) == system.nInputs
        u_k = simPars.uConst;
    else
        u_k = zeros(system.nInputs, 1);
    end

    % Time-varying inputs
    if (~isempty(simPars.uSampleValues) && size(simPars.uSampleValues,1) == system.nInputs ) && ...
            ~isempty(simPars.uSampleTimes) && ...
            size(simPars.uSampleValues,2) == size(simPars.uSampleTimes,1)
        u_k = u_k + interp1(simPars.uSampleTimes, simPars.uSampleValues.', t, 'linear', 0).';
    end

    %% Evaluate EOM

    % Generalized forces (stress, dissipation and system inputs)
    f_gen = system.cSys .* (q - system.qRef) ...
        + system.dSys .* q_dot ...
        - system.computeInputMatrix(q) * u_k;

    % External frame forces from the environment
    f_frame_b = simPars.externalWrench_b.getCurrentWrench(system.nFrames, t);
    f_frame_s = simPars.externalWrench_s.getCurrentWrench(system.nFrames, t);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + elara.dynamics.num.bodyFixedFrameForces(g, f_frame_s, system, simPars);

    % Compute sum of generalized forces
    for iFrm = 1:system.nFrames
        f_gen = f_gen + J(:,:,iFrm).' * (...
            ...% Frame forces
            + f_frame_b(:,iFrm)...
            ...% Coriolis Term
            + (system.frames.MGen(:,:,iFrm) * J_dot(:,:,iFrm) ...
            - elara.SE3.smallAd(eta(:,iFrm)).' * system.frames.MGen(:,:,iFrm) * J(:,:,iFrm)) * q_dot ...
            );
    end

    %% Assemble first-order RHS
    f_fo = [q_dot; -f_gen];

end
