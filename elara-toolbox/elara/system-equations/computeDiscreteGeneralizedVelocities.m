function q_dot_k = computeDiscreteGeneralizedVelocities(obj, g_rel_k, g_rel_k1, h)
    %% Compute discrete generalized velocities in the interval (k,k+1)
    % i.e., compute generalized (relative) velocities q_dot in
    % R^nDof from given relative transformations at time
    % instances k and k+1
    arguments
        obj         (1,1)   MBSystemNum

        % Array of relative configurations at time step k
        g_rel_k     (4,4,:) double

        % Array of relative configurations at time step k+1
        g_rel_k1    (4,4,:) double

        % Time step
        h           (1,1)   double
    end
    % Vector of generalized velocities
    q_dot_k = zeros(obj.nDoF, 1);

    % Computation for all frames
    for iFrm = 1:obj.nFrames
        qIndices = obj.frameData.getQIndices(iFrm);

        % Compute relative transformation matrix of frame update
        % transformation over time step
        g_rel = invSE3Matrix(g_rel_k(:,:,iFrm)) * g_rel_k1(:,:,iFrm);

        % Compute gen. velocity based on joint type
        switch obj.frameData.jointType(iFrm)
            case 1
                %%% Screw joint
                % Check screw axis to determine from which part of
                % g the velocity is computed
                if any(obj.frameData.X(1:3,iFrm))
                    om = obj.frameData.X(1:3,iFrm);
                    q_dot_k(qIndices) = om.'*cayInvSO3(g_rel(1:3,1:3)) / (om.'*om)/h;
                else
                    % Compute from translational part
                    % (solve x in SE3 exponential for q)
                    om = obj.frameData.X(1:3,iFrm);
                    v  = obj.frameData.X(4:6,iFrm);
                    rightTerm = g_rel(1:3,4) - (eye(3) - g_rel(1:3,1:3))*skewSO3(om)*v;
                    q_dot_k(qIndices) = ((om*om.'*v).' * rightTerm) / ( (om*om.'*v).'*(om*om.'*v))/h;
                    warning("Computation not verified yet");
                end
            case 2
                %%% Flexible joint
                Ba = obj.frameData.Ba(iFrm);
                q_dot_k(qIndices) = Ba.' * cayInvSE3(g_rel)/h/obj.frameData.l(iFrm);
            otherwise
                error("Invalid joint type specified.");
        end
    end
end
