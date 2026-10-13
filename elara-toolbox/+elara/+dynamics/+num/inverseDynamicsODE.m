function [u, solInfo] = inverseDynamicsODE(system, simPars, q, qd, qdd, lbu, ubu)
    %% Compute inverse dynamics for given coordinate trajectory
    % using continuous-time equations of motion (ODEs)
    arguments
        system   (1,1) elara.SystemNum
        simPars (1,1) elara.SimulationParameters

        % Trajectory over time with dimensions (nDoF, nSteps+1)
        q       (:,:) double
        qd      (:,:) double
        qdd     (:,:) double

        % Upper and lower bounds for u
        % Only considered for underactuated systems
        lbu      (:,1) double
        ubu      (:,1) double
    end

    % Number of time steps
    nSteps = size(q, 2) - 1;

    % Zero input vector used in the dynamics functions
    uZero = zeros(system.nInputs, 1);

    u = nan(system.nInputs, nSteps+1);

    solInfo.resNorm  = zeros(nSteps + 1,1);
    solInfo.rank_B = zeros(nSteps + 1,1);
    solInfo.cond_B = zeros(nSteps + 1,1);

    % Check whether system is fully actuated or underactuated
    g_rel_1 = system.computeFwdKin(q(:,1));
    isFullyActuated = rank(system.computeInputMatrix(q(:,1), g_rel_1)) == system.nDoF;

    %% Compute Inputs
    for k = 1:nSteps+1
        res_k = elara.dynamics.num.secondOrderODEResidual(0, q(:,k), qd(:,k), qdd(:,k), uZero, system, simPars);
        g_rel_k = system.computeJointTransformations(q(:,k));

        B_k = system.computeInputMatrix(q(:,k), g_rel_k);
        % Compute Inputs
        if isFullyActuated
            % Fully actuated system: Directly invert input matrix
            u(:,k) = -B_k \ res_k;
        else
            [u(:,k), solInfo_k] = elara.internal.dynamics.solveSystemInputs( ...
                system, res_k, q(:,k), lbu, ubu);
            solInfo.resNorm(k)  = solInfo_k.resNorm;
            solInfo.rank_B(k) = rank(B_k);
            solInfo.cond_B(k) = cond(B_k);
        end
    end
end
