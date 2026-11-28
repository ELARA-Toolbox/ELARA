function f_fo = firstOrderDerivative(x, u, system, simPars)
    %% Compute the Right-Hand Side of the EOMs in first-order form (with inverted Mass Matrix)
    arguments (Input)
        % State vector x = [q; q_dot] (2*nDoF, 1)
        x       (:,1)

        % Input vector
        u       (:,1)

        system  (1,1) elara.SystemSym

        simPars (1,1) elara.SimulationParameters
    end
    arguments (Output)
        f_fo        (:,1)
    end

    % Get right-hand side of the mass-matrix form of the equations
    rhs = elara.dynamics.sym.firstOrderRHS(x, system, simPars, u);

    M_C = system.computeMassMatrix(x(1:system.nDoF));
    M_rows = cell(system.nFrames,1);
    for iFrm = 1:system.nFrames
        M_rows{iFrm} = horzcat(M_C{iFrm,:});
    end
    M = vertcat(M_rows{:});

    if isa(rhs, "casadi.MX")
        % MX dynamics require symbolicqr for expansion to SX in NLP solvers,
        % See https://github.com/casadi/casadi/wiki/FAQ:-how-to-resolve-'eval_sx'-not-defined%3F
        acceleration = solve(M, rhs(system.nDoF+1:end), 'symbolicqr');
    else
        % Numeric and SX inputs use their native matrix-division overloads.
        acceleration = M \ rhs(system.nDoF+1:end);
    end
    f_fo = [rhs(1:system.nDoF); acceleration];
end
