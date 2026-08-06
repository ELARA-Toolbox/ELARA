function [res_0, eta_0] = DELResidualInitialStep(system, h, simPars, q_0, q_1, u_0, a)
    %% Compute the Residuum for the first step in DEL integration
    % i.e., step 0 -> 1,
    % according to the formula in [Obe08, p.50]
    % based on the Legendre transform to properly include initial velocities
    % Notes:
    %  * We're using the complete trapezoidal rule for the first step
    %    for true second-order accuracy
    arguments
        system  (1,1) elara.abstract.System
        h       (1,1)
        simPars (1,1) elara.SimulationParameters
        q_0     (:,1) double % Coordinate variable at time step k = 1
        q_1     (:,1) double % Coordinate variable at time step k = 2
        u_0     (:,1) double % System input variable at time step k = 1

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double
    end

    % Forward kinematics for the first two steps (k = 1, 2)
    [g_0, g_rel_0] = system.computeFwdKin(q_0);
    g_rel_1 = system.computeJointTransformations(q_1);

    % Velocities
    eta_0 = system.computeDiscreteAbsoluteVelocities(g_rel_0,  g_rel_1, h);

    % Evaluate DEL
    res_0 = elara.dynamics.num.DELResidualInitialStep_noKinematics(system, h, ...
        simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, simPars.qDot0, a);
end
