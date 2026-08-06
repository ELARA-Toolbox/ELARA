function M_fo = firstOrderMassMatrix(t, x, system)
    %% Compute the overall mass matrix for a multibody system in first-order form
    % I.e., the mass matrix diag(I, M) with dimension 2*nDof x 2*nDof
    arguments (Input)
        % Integration time (from ode solver)
        % Not needed for the function, but "~" is not allowed for codegen
        t       (1,1)

        % State vector [q; q_dot] (2*nDof,1)
        x       (:,1)

        system  (1,1) elara.abstract.System
    end

    M_fo = blkdiag( ...
        eye(system.nDoF), ...
        computeSystemMassMatrix([], x, system) ...
        );

end
function [M, J] = computeSystemMassMatrix(~, x, system)
    %% Compute the system mass matrix of a multibody system
    arguments (Input)
        % Integration time (from ode solver)
        ~%t

        % State vector [q; q_dot] (2*nDof,1)
        x      (:,1)

        system (1,1) elara.abstract.System
    end
    arguments (Output)
        % System mass matrix
        M   (:,:)

        % Array of frame Jacobian matrices
        J   (:,1) cell
    end

    % Get Jacobians
    J = system.computeGeomJacobian( x(1:numel(x)/2) );

    % Mass matrix
    if isa(x, "casadi.MX")
        M = casadi.MX.zeros(system.nDoF);
    elseif isa(x, "casadi.SX")
        M = casadi.SX.zeros(system.nDoF);
    else
        M = zeros(system.nDoF, class(x));
    end
    for iFrm = 1:system.nFrames
        M = M + J{iFrm}.' * system.frames.MGen(:,:,iFrm) * J{iFrm};
    end
end
