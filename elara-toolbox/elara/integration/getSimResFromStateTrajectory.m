function simRes = getSimResFromStateTrajectory(system, tout, q, q_dot)
    %% Get simRes object from trajectory of states q, q_dot
    % E.g., to post-process integration results from ODE integrator
    arguments
        % Object defining the multibody system
        system  (1,1) elara.SystemNum

        % Time vector
        tout    (:,1) double

        % Matrix of configurations at time steps tout,
        % dimensions (nDoF, nSteps)
        q       (:,:) double

        % Matrix of velocities at time steps tout
        % dimensions (nDoF, nSteps)
        q_dot   (:,:) double;
    end

    % Check dimensions of configurations and velocities
    assert(size(q,1) == system.nDoF, "Matrix of Configurations has wrong dimensions");
    assert(size(q_dot,1) == system.nDoF, "Matrix of Velocities has wrong dimensions");

    nSteps = numel(tout);
    eta    = zeros(6, system.nFrames, nSteps);
    g      = zeros(4,4, system.nFrames, nSteps);

    for iStep = 1:nSteps
        % Forward kinematics and absolute velocities
        [g(:,:,:,iStep), g_rel] = system.computeFwdKin(q(:,iStep));
        J = system.computeGeomJacobianFast( q(:,iStep), g_rel);
        for iFrm = 1:system.nFrames
            eta(:,iFrm,iStep) = J(:,:,iFrm) * q_dot(:,iStep);
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