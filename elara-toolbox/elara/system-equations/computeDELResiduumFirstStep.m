function [res_0, eta_0] = computeDELResiduumFirstStep(MBSys, h, simPars, q_0, q_1, u_0, a, MGenCell)
    %% Compute the Residuum for the first step in DEL integration
    % i.e., step 0 -> 1,
    % according to the formula in [Obe08, p.50]
    % based on the Legendre transform to properly include initial velocities
    % Notes:
    %  * We're using the complete trapezoidal rule for the first step
    %    for true second-order accuracy
    arguments
        MBSys   (1,1) elara.internal.System
        h       (1,1)
        simPars (1,1) MBSimPars
        q_0     (:,1) double % Coordinate variable at time step k = 1
        q_1     (:,1) double % Coordinate variable at time step k = 2
        u_0     (:,1) double % System input variable at time step k = 1

        % Weighting factor in the generalized trapezoidal rule
        a       (1,1) double

        % Cell array of generalized mass matrices
        % Specified as an extra argument to use generalized mass matrices
        % with symbolic variables (for parameter identification)
        MGenCell (:,1) cell = {}
    end

    % Get generalized mass matrix array from numeric array if not given
    if isempty(MGenCell)
        MGenCell = squeeze(num2cell(MBSys.frames.MGen,[1,2]));
    end

    % Forward kinematics for the first two steps (k = 1, 2)
    [g_0, g_rel_0] = MBSys.computeFwdKin(q_0);
    [~,   g_rel_1] = MBSys.computeFwdKin(q_1);

    % Velocities
    eta_0 = MBSys.computeDiscreteAbsoluteVelocities(g_rel_0,  g_rel_1, h);

    % Evaluate DEL
    res_0 = computeDELResiduumFirstStep_extKin(MBSys, h, simPars, q_0, q_1, g_0, g_rel_0, eta_0, u_0, simPars.qDot0, a, MGenCell);
end