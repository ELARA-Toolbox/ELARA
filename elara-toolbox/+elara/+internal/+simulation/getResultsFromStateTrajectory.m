function simRes = getResultsFromStateTrajectory(system, tout, q, q_dot)
    %% Create simulation results from configuration and velocity trajectories
    arguments
        % Object defining the multibody system
        system  (1,1) elara.SystemNum

        % Time vector
        tout    (:,1) double

        % Configurations at the times in tout, (nDoF, nTimes)
        q       (:,:) double

        % Generalized velocities at the times in tout, (nDoF, nTimes)
        q_dot   (:,:) double;
    end

    % Check dimensions of configurations and velocities
    nTimes = numel(tout);
    assert(size(q,1) == system.nDoF && size(q,2) == nTimes, ...
        "The configuration trajectory must have size nDoF-by-nTimes.");
    assert(size(q_dot,1) == system.nDoF && size(q_dot,2) == nTimes, ...
        "The velocity trajectory must have size nDoF-by-nTimes.");

    eta = zeros(6, system.nFrames, nTimes);
    g   = zeros(4,4, system.nFrames, nTimes);

    for iTime = 1:nTimes
        % Forward kinematics and absolute velocities
        [g(:,:,:,iTime), g_rel] = system.computeFwdKin(q(:,iTime));
        J = system.computeGeomJacobianFast(q(:,iTime), g_rel);
        for iFrm = 1:system.nFrames
            eta(:,iFrm,iTime) = J(:,:,iFrm) * q_dot(:,iTime);
        end
    end

    % Assign to object
    simRes = elara.SimulationResults;
    simRes.tout  = tout;
    simRes.q     = q;
    simRes.q_dot = q_dot;
    simRes.eta   = eta;
    simRes.g     = g;
end
