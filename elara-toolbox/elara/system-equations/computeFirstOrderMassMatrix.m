function M_fo = computeFirstOrderMassMatrix(t, x, MBSys)
    %% Compute the overall mass matrix for a multibody system in first-order form
    % I.e., the mass matrix diag(I, M) with dimension 2*nDof x 2*nDof
    arguments (Input)
        % Integration time (from ode solver)
        % Not needed for the function, but "~" is not allowed for codegen
        t           (1,1) double

        % State vector [q; q_dot] (2*nDof,1)
        x       (:,1) double

        MBSys   (1,1) elara.SystemNum
    end
    M_fo = blkdiag( ...
        eye(MBSys.nDoF), ...
        MBSys.computeMassMatrix(x(1:numel(x)/2)) ...
        );
end
