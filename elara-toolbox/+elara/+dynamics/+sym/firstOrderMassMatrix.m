function M_fo = firstOrderMassMatrix(t, x, system)
    %% Compute the first-order mass matrix diag(I, M(q))
    arguments (Input)
        % Integration time (unused; retained for the ODE function interface)
        t       (1,1)

        % State vector x = [q; q_dot] (2*nDoF, 1)
        x       (:,1)

        system  (1,1) elara.SystemSym
    end

    % Preserve the dimensions of every coordinate block, including the
    % zero blocks between uncoupled branches.
    M_C = system.computeMassMatrix(x(1:system.nDoF));
    M_rows = cell(system.nFrames,1);
    for iFrm = 1:system.nFrames
        M_rows{iFrm} = horzcat(M_C{iFrm,:});
    end
    M = vertcat(M_rows{:});
    M_fo = blkdiag(eye(system.nDoF), M);
end
