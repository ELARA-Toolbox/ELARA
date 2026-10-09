function [u, solInfo] = inverseDynamicsDEL(system, simPars, q, qd, h, lbu, ubu)
    %% Compute inverse dynamics for a configuration trajectory using the discrete Euler-Lagrange equations
    arguments
        system  (1,1) elara.SystemNum
        simPars (1,1) elara.SimulationParameters

        % Trajectory over time with dimensions (nDoF, nSteps+1)
        q       (:,:) double % Configuration q(t)
        qd      (:,:) double % Generalized velocity qDot(t)

        % Time step size
        h       (1,1) double

        % Upper and lower bounds for u
        % Only considered for underactuated systems
        lbu      (:,1) double
        ubu      (:,1) double
    end

    % Number of time steps
    nSteps = size(q, 2) - 1;

    % Output time vector
    tout = 0:h:h*nSteps;

    % Zero input vector used in the DEL functions
    uZero = zeros(system.nInputs, 1);

    u = nan(system.nInputs, nSteps+1);
    solInfo.resNorm = zeros(nSteps+1,1);
    solInfo.rank_B  = zeros(nSteps+1,1);
    solInfo.cond_B  = zeros(nSteps+1,1);

    % Check whether system is fully actuated or underactuated

    g_rel_k  = system.computeJointTransformations(q(:,1));

    B_1 = system.computeInputMatrix(q(:,1), g_rel_k);
    isFullyActuated = rank(B_1) == system.nDoF;


    %% Initial step

    % Gen. trapezoidal rule coefficient
    a = 1/2;

    % DEL residual
    [g_k]  = system.computeFwdKinFast(g_rel_k);
    [g_k1, g_rel_k1] = system.computeFwdKin(q(:,2));
    eta_k = system.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);
    res_0 = elara.dynamics.num.DELResidualInitialStep_noKinematics(system, h, simPars, ...
        q(:,1), q(:,2), g_k, g_rel_k, eta_k, uZero, qd(:,1), a);

    
    % Compute Inputs
    if isFullyActuated
        % Fully actuated system: Directly invert input matrix
        u(:,1) = ((1-a)*B_1) \ res_0;
    else
        % Underactuated system
        % Note: We directly combine the trapezoidal rule factor (1-a) in the
        % first step with the time step h
        [u(:,1), solInfo_1] = elara.internal.dynamics.solveSystemInputs( ...
            system, res_0/(1-a), q(:,1), lbu, ubu);

        solInfo.resNorm(1)  = solInfo_1.resNorm;
        solInfo.rank_B(1) = rank(B_1, g_rel_k));
        solInfo.cond_B(1) = cond(B_1, g_rel_k));
    end

    %% Intermediate steps
    for k = 2:nSteps
        eta_k0 = eta_k;
        g_k = g_k1;
        g_rel_k = g_rel_k1;

        [g_k1, g_rel_k1] = system.computeFwdKin(q(:,k+1));
        eta_k = system.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);

        % External frame forces from the environment
        f_frame_k_b_ext = simPars.externalWrench_b.getCurrentWrench(system.nFrames, tout(k));
        f_frame_k_s_ext = simPars.externalWrench_s.getCurrentWrench(system.nFrames, tout(k));

        % Get residual
        res_k = elara.dynamics.num.DELResidual_noKinematics(system, simPars, ...
            q(:,k-1), q(:,k), q(:,k+1), g_k, g_rel_k, eta_k, eta_k0, ...
            uZero, f_frame_k_b_ext, f_frame_k_s_ext, h, a);

        B_k = system.computeInputMatrix(q(:,k), g_rel_k);

        % Compute Inputs
        if isFullyActuated
            u(:,k) = -B_k \ res_k;
        else
            [u(:,k), solInfo_k] = elara.internal.dynamics.solveSystemInputs( ...
                system, res_k, q(:,k), lbu, ubu);

            solInfo.resNorm(k)  = solInfo_k.resNorm;
            solInfo.rank_B(k) = rank(B_k, g_rel_k);
            solInfo.cond_B(k) = cond(B_k, g_rel_k);
        end
    end

    %% Final step
    res_N1 = elara.dynamics.num.DELResidualFinalStep( ...
        system, h, simPars, q(:,nSteps), q(:,nSteps+1), uZero, qd(:,nSteps+1), a);

    g_rel_k1 = system.computeJointTransformations(q(:,nSteps+1));
    B_k1 = system.computeInputMatrix(q(:,nSteps+1), g_rel_k1);

    % Compute Inputs
    if isFullyActuated
        u(:,nSteps+1) = -(a*B_k1) \ res_N1;
    else
        [u(:,nSteps+1), solInfo_k] = elara.internal.dynamics.solveSystemInputs( ...
            system, res_N1/a, q(:,nSteps+1), lbu, ubu);
        solInfo.resNorm(nSteps+1)  = solInfo_k.resNorm;
        solInfo.rank_B(nSteps+1) = rank(B_k1);
        solInfo.cond_B(nSteps+1) = cond(B_k1);
    end
end
