function [q, u] = computeStaticEquilibrium(problem)
    %% Compute a static equilibrium for the problem's desired TCP position
    % The equilibrium satisfies the static force balance, workspace
    % constraints, control bounds, and configured final TCP objective or
    % constraint of the optimal-control problem.
    arguments
        problem (1,1) elara.ocp.Problem
    end

    [q, u] = elara.internal.ocp.computeTCPSteadyState(problem);
end
