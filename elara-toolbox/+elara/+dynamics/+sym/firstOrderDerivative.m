function f_fo = firstOrderDerivative(t, x, u, system, simPars) %#codegen
    %% Compute the Right-Hand Side of the EOMs in first-order form (with inverted Mass Matrix)
    arguments (Input)
        % Integration time (from ode solver)
        t       (1,1)

        % State vector [q; q_dot] (2*nDof,1)
        x       (:,1)

        % Input vector
        u       (:,1)

        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters
    end
    arguments (Output)
        f_fo        (:,1)
    end

    f = elara.internal.math.getSE3Functions(x);

    %% Get configuration and velocity
    q     = x(1:system.nDoF);
    q_dot = x((system.nDoF+1):end);

    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = system.computeFwdKin(q);
    J = system.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    eta = cell(system.nFrames,1);
    for iFrm = 1:system.nFrames
        for iBlock = 1:system.nFrames
            qIndices = double( system.frames.qIndices(1,iBlock):system.frames.qIndices(2,iBlock));
            if ~isempty(J{iFrm, iBlock})
                if isempty(eta{iFrm})
                    eta{iFrm} = J{iFrm, iBlock} * q_dot(qIndices);
                else
                    eta{iFrm} = eta{iFrm} + J{iFrm, iBlock} * q_dot(qIndices);
                end
            end
        end
    end
    J_dot = system.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);

    %% Evaluate EOM

    % Generalized forces (stress, dissipation and system inputs)
    f_gen = system.cSys .* (q - system.qRef) ...
        + system.dSys .* q_dot;

    % Input term
    f_gen_C = cell(system.nFrames, 1);
    B = system.computeInputMatrix(q);
    for iFrm = 1:system.nFrames
        for iInput = 1:system.nInputs
            if ~isempty(B{iFrm, iInput})
                if isempty(f_gen_C{iFrm})
                    f_gen_C{iFrm} = -B{iFrm, iInput} * u(iInput);
                else
                    f_gen_C{iFrm} = f_gen_C{iFrm} - B{iFrm, iInput} * u(iInput);
                end
            end
        end
    end

    % Placeholder values for external forces
    f_frame_b = zeros(6, system.nFrames);
    f_frame_s = zeros(6, system.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b_C = elara.dynamics.sym.bodyFixedFrameForces(system, g, f_frame_s, simPars.g);

    % Compute J_dot * eta
    JdotTerm = cell(system.nFrames,1);
    for iFrm = 1:system.nFrames
        for iBlock = 1:system.nFrames
            qIndices = double( system.frames.qIndices(1,iBlock):system.frames.qIndices(2,iBlock));
            if ~isempty(J_dot{iFrm, iBlock})
                if isempty(JdotTerm{iFrm})
                    JdotTerm{iFrm} = J_dot{iFrm, iBlock} * q_dot(qIndices);
                else
                    JdotTerm{iFrm} = JdotTerm{iFrm} + J_dot{iFrm, iBlock} * q_dot(qIndices);
                end
            end
        end
    end

    for iFrm = 1:system.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            - f_frame_b(:, iFrm) ...
            + f_frame_b_C{iFrm} ...
            ...% Coriolis Term
            + system.frames.MGen{iFrm} * JdotTerm{iFrm} ...
            - f.SE3.smallAd(eta{iFrm}(1:3),eta{iFrm}(4:6)).' * system.frames.MGen{iFrm} * eta{iFrm};
        % Distribute node terms to coordinates
        for iB = 1:system.nFrames
            if ~isempty(J{iFrm,iB})
                if isempty(f_gen_C{iB})
                    f_gen_C{iB} = J{iFrm,iB}.' * f_frame_b_i;
                else
                    f_gen_C{iB} = f_gen_C{iB} + J{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    f_gen = vertcat(f_gen_C{:}) + f_gen;

    %% Assemble first-order RHS
    M_C = system.computeMassMatrix(q);
    M_rows = cell(system.nFrames,1);
    for iFrm = 1:system.nFrames
        M_rows{iFrm} = horzcat(M_C{iFrm,:});
    end
    M = vertcat(M_rows{:});
    % Note: Additional argument 'symbolicqr' required to allow expansion of
    % the MX graph to SX when defining an NLP solver with "expand = true"
    % See https://github.com/casadi/casadi/wiki/FAQ:-how-to-resolve-'eval_sx'-not-defined%3F
    f_fo = [q_dot; -inv(M, 'symbolicqr')*f_gen];
end
