function res = residual(system, simPars, q, u)
    %% Compute the static-equilibrium residual
    arguments
        % Multibody system
        system  (1,1) elara.SystemNum

        simPars (1,1) elara.SimulationParameters

        % Generalized coordinates (nDoF, 1)
        q       (:,1) double

        % System inputs (nInputs, 1)
        u       (:,1) double
    end

    % Check argument sizes
    assert( size(q,1) == system.nDoF, ...
        "The generalized-coordinate vector must contain nDoF elements.");
    assert( size(u,1) == system.nInputs, ...
        "The input vector must contain nInputs elements.");

    % Forward Kinematics and Jacobians
    [g, g_rel] = system.computeFwdKin(q);
    J = system.computeGeomJacobianFast(q, g_rel);

    % Generalized forces (stress and inputs)
    f_gen = system.cSys .* (q - system.qRef) - system.computeInputMatrixFast(g_rel) * u;

    % Placeholder values for external forces
    f_frame_b = zeros(6, system.nFrames);
    f_frame_s = zeros(6, system.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + elara.dynamics.num.bodyFixedFrameForces(g, f_frame_s, system, simPars);

    % Compute sum of generalized forces
    res = f_gen;
    for iFrm = 1:system.nFrames
        res = res + J(:,:,iFrm).' * f_frame_b(:,iFrm);
    end

end
