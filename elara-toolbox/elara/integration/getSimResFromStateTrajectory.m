function simRes = getSimResFromStateTrajectory(MBSys, tout, q, q_dot)
    %% Get simRes object from trajectory of states q, q_dot
    % E.g., to post-process integration results from ODE integrator
    arguments
        % Object defining the multibody system
        MBSys   (1,1) MBSystemNum

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
    assert(size(q,1) == MBSys.nDoF, "Matrix of Configurations has wrong dimensions");
    assert(size(q_dot,1) == MBSys.nDoF, "Matrix of Velocities has wrong dimensions");

    nSteps = numel(tout);
    eta    = zeros(6, MBSys.nFrames, nSteps);
    g      = zeros(4,4, MBSys.nFrames, nSteps);

    for iStep = 1:nSteps
        % Forward kinematics and absolute velocities
        [g(:,:,:,iStep), g_rel] = MBSys.computeFwdKin(q(:,iStep));
        J = MBSys.computeGeomJacobianFast( q(:,iStep), g_rel);
        for iFrm = 1:MBSys.nFrames
            eta(:,iFrm,iStep) = J(:,:,iFrm) * q_dot(:,iStep);
        end
    end

    % Assign to object
    simRes = MBSimResults;
    simRes.tout  = tout;
    simRes.q     = q;
    simRes.q_dot = q_dot;
    simRes.eta   = eta;
    simRes.g     = g;
end