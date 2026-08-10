function M_fo = firstOrderMassMatrix(t, x, system)
    %% Compute the overall mass matrix for a multibody system in first-order form
    % That is, the mass matrix diag(I, M) has size 2*nDoF-by-2*nDoF.
    arguments (Input)
        % Integration time (from ode solver)
        % Not needed for the function, but "~" is not allowed for codegen
        t       (1,1) double

        % State vector x = [q; q_dot] (2*nDoF, 1)
        x       (:,1) double

        system  (1,1) elara.SystemNum
    end
    M_fo = blkdiag( ...
        eye(system.nDoF), ...
        system.computeMassMatrix(x(1:numel(x)/2)) ...
        );
end
