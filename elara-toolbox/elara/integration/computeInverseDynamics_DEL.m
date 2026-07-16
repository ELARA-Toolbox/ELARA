function [u, solInfo] = computeInverseDynamics_DEL(MBSim, q, qd, h, lbu, ubu)
    %% Compute inverse dynamics for given coordinate trajectory using DEL Equs.
    arguments
        MBSim   (1,1) elara.Simulation

        % Trajectory over time with dimensions (nDoF, nSteps+1)
        q       (:,:) double % Configuration q(t)
        qd      (:,:) double % Generalized velocity qDot(t)

        % Time step size
        h       (1,1) double

        % Upper and lower bounds for u
        lbu      (:,1) double
        ubu      (:,1) double
    end

    MBSys = MBSim.system;
    simPars = MBSim.parameters;
   
    % Nr. of time steps
    nSteps = size(q, 2) - 1;

    % Output time vector
    tout = 0:h:h*nSteps;

    % Zero input vector used in the DEL functions
    uZero = zeros(MBSys.nInputs, 1);

    u = nan(MBSys.nInputs, nSteps+1);
    solInfo.resNorm = zeros(nSteps+1,1);
    solInfo.rank_B  = zeros(nSteps+1,1);
    solInfo.cond_B  = zeros(nSteps+1,1);

    % Check whether system is fully actuated or underactuated
    isFullyActuated = rank(MBSys.computeInputMatrix(q(:,1))) == MBSys.nDoF;


    %% Initial step

    % Gen. trapezoidal rule coefficient
    a = 1/2;

    % DEL residual
    [g_k,  g_rel_k]  = MBSys.computeFwdKin(q(:,1));
    [g_k1, g_rel_k1] = MBSys.computeFwdKin(q(:,2));
    eta_k = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);
    res_0 = computeDELResiduumFirstStep_extKin(MBSys, h, simPars, ...
        q(:,1), q(:,2), g_k, g_rel_k, eta_k, uZero, qd(:,1), a);

    % Compute Inputs
    if isFullyActuated
        % Fully actuated system: Directly invert input matrix
        u(:,1) = ((1-a)*MBSys.computeInputMatrix(q(:,1))) \ res_0;
    else
        % Underactuated system
        % Note: We directly combine the trapezoidal rule factor (1-a) in the
        % first step with the time step h
        [u(:,1), solInfo_1] = solveEOMInputs(MBSys, res_0/(1-a), q(:,1), lbu, ubu);

        solInfo.resNorm(1)  = solInfo_1.resNorm;
        %solInfo.residual(1) = solInfo_1.residual;
        solInfo.rank_B(1) = rank(MBSys.computeInputMatrix(q(:,1)));
        solInfo.cond_B(1) = cond(MBSys.computeInputMatrix(q(:,1)));
    end

    %% Intermediate steps
    for k = 2:nSteps
        eta_k0 = eta_k;
        g_k = g_k1;
        g_rel_k = g_rel_k1;

        [g_k1, g_rel_k1] = MBSys.computeFwdKin(q(:,k+1));
        eta_k = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);

        % External frame forces from the environment
        f_frame_k_b_ext = getExternalStepWrenches(simPars.extWrench_b, MBSys.nFrames, tout(k));
        f_frame_k_s_ext = getExternalStepWrenches(simPars.extWrench_s, MBSys.nFrames, tout(k));

        % Get residuum
        res_k = computeDELResiduum_extKin(MBSys, simPars, ...
            q(:,k-1), q(:,k), q(:,k+1), g_k, g_rel_k, eta_k, eta_k0, ...
            uZero, f_frame_k_b_ext, f_frame_k_s_ext, h, a);

        % Compute Inputs
        if isFullyActuated
            u(:,k) = -MBSys.computeInputMatrix(q(:,k)) \ res_k;
        else
            [u(:,k), solInfo_k] = solveEOMInputs(MBSys, res_k, q(:,k), lbu, ubu);

            solInfo.resNorm(k)  = solInfo_k.resNorm;
            %solInfo.residual(k) = solInfo_k.residual;
            solInfo.rank_B(k) = rank(MBSys.computeInputMatrix(q(:,k)));
            solInfo.cond_B(k) = cond(MBSys.computeInputMatrix(q(:,k)));
        end
    end

    %% Final step
    res_N1 = computeDELResiduumFinalStep( ...
        MBSys, h, simPars, q(:,nSteps), q(:,nSteps+1), uZero, qd(:,nSteps+1), a);

    % Compute Inputs
    if isFullyActuated
        u(:,nSteps+1) = -(a*MBSys.computeInputMatrix(q(:,nSteps+1))) \ res_N1;
    else
        [u(:,nSteps+1), solInfo_k] = solveEOMInputs(MBSys, res_N1/a, q(:,nSteps+1), lbu, ubu);
        solInfo.resNorm(nSteps+1)  = solInfo_k.resNorm;
        solInfo.rank_B(nSteps+1) = rank(MBSys.computeInputMatrix(q(:,nSteps+1)));
        solInfo.cond_B(nSteps+1) = cond(MBSys.computeInputMatrix(q(:,nSteps+1)));
    end
end