function f_fo = firstOrderRHS(t, x, system, simPars, u)
    %% Compute the Right-Hand Side of the EOMs in first-order form
    arguments (Input)
        % Integration time (from ode solver)
        % Not needed for the function, but "~" is not allowed for codegen
        t           (1,1)

        % State vector x = [q; q_dot] (2*nDoF, 1)
        x           (:,1)

        system      (1,1) elara.abstract.System

        simPars     (1,1) elara.SimulationParameters

        % Vector of input variables
        u           (:,1)
    end
    arguments (Output)
        f_fo        (:,1)
    end

    %% Get configuration and velocity
    q     = x(1:system.nDoF);
    q_dot = x((system.nDoF+1):end);


    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = system.computeFwdKin(q);
    J = system.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    if isa(x, "casadi.MX")
        eta = casadi.MX.zeros(6,system.nFrames);
    elseif isa(x, "casadi.SX")
        eta = casadi.SX.zeros(6,system.nFrames);
    else
        eta = zeros(6,system.nFrames, class(x));
    end
    for iFrm = 1:system.nFrames
        eta(:,iFrm) = J{iFrm} * q_dot;
    end
    J_dot = system.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);


    %% Evaluate EOM

    % Generalized forces (stress and dissipation)
    f_gen = system.cSys .* (q - system.qRef) + system.dSys .* q_dot;

    % Actuation (if nonzero)
    if ~isempty(simPars.uConst)
        f_gen = f_gen + system.computeInputMatrix(q) * u;
    end

    % Placeholder values for external forces
    f_frame_b = zeros(6, system.nFrames);
    f_frame_s = zeros(6, system.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + elara.dynamics.sym.bodyFixedFrameForces(system, g, f_frame_s, simPars.g);

    % Compute sum of generalized forces
    for iFrm = 1:system.nFrames
        f_gen = f_gen + J{iFrm}.' * (...
            ...% Frame forces
            + f_frame_b(:,iFrm)...
            ...% Coriolis Term
            + (system.frames.MGen(:,:,iFrm) * J_dot{iFrm} ...
            - elara.SE3.smallAd(eta(:,iFrm)).' * system.frames.MGen(:,:,iFrm) * J{iFrm}) * q_dot ...
            );
    end

    %% Assemble first-order RHS
    f_fo = [q_dot; -f_gen];

end
