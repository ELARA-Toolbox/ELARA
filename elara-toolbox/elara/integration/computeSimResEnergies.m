function simEnergies = computeSimResEnergies(MBSys, simPars, simRes, isVarInt, useFD) %#codegen
    %% Compute energy evolution for simulation results
    arguments (Input)
        MBSys       (1,1) elara.SystemNum
        simPars     (1,1) MBSimPars
        simRes      (1,1) MBSimResults

        % Defines whether simulation results belong to a variational
        % integrator
        isVarInt    (1,1) logical

        % For varInts: Whether to compute the kinetic energy using generalized
        % velocities computed from finite differences (true) or via
        % discrete momenta
        useFD       (1,1) logical
    end

    nSteps = numel(simRes.tout);
    T = zeros(nSteps, 1);
    U = zeros(nSteps, 1);
    V = zeros(nSteps, 1);

    % Trapezoidal rule factor
    a = 1/2;

    % Get system inputs for varInts
    if isVarInt
        u = getIntegratorInputs(MBSys, simPars, simRes.tout);
        h = simRes.tout(2) - simRes.tout(1);

        % Get velocities via finite differences
        if useFD
            q_dot = diff2ndOrder(simRes.q, h);
        else
            q_dot = [];
        end
    else
        % Define dummy variables for codegen
        u = [];
        h = 1;

        % Get velocities
        q_dot = simRes.q_dot;
    end

    for iStep = 1:nSteps
        q_k = simRes.q(:,iStep);
        g_k = simRes.g(:,:,:,iStep);

        % Kinetic energy
        if isVarInt && ~useFD
            % Variational integrators: Compute kinetic energy at
            % time nodes based on discrete momenta
            if iStep < nSteps
                % Steps 1, ..., N-1: Left momentum
                q_k1 = simRes.q(:,iStep+1);
                p_k = computeLeftGeneralizedMomentum(MBSys, h, simPars, ...
                    q_k, q_k1, g_k, simRes.eta(:,:,iStep), u(:,iStep), a);
            else
                % Last step: Right momentum
                q_k0 = simRes.q(:,iStep-1);
                p_k = computeRightGeneralizedMomentum(MBSys, h, simPars, ...
                    q_k0, q_k, g_k, simRes.eta(:,:,iStep-1), u(:,iStep), a);
            end
            T(iStep) = 1/2 * p_k.' / MBSys.computeMassMatrix(q_k) * p_k;
        else
            % ODE integrators: Directly compute kinetic energy from
            % time node velocities
            q_dot_k = q_dot(:,iStep);
            T(iStep) = 1/2 * q_dot_k.' * MBSys.computeMassMatrix(q_k) * q_dot_k;
        end

        for iFrm = 1:MBSys.nFrames
            % Potential energy (both frame CoM and attached masses)
            g_a_k = g_k(:,:,iFrm) * MBSys.frameData.g_a(:,:,iFrm);
            U(iStep) = U(iStep) + simPars.g *(...
                MBSys.frameData.m(iFrm) * g_k(3,4,iFrm) + ...
                MBSys.frameData.m_a(iFrm) * g_a_k(3,4)...
                );
        end

        % Strain energy
        V(iStep) = 1/2 * (q_k - MBSys.qRef).' ...
            * (MBSys.cSys .* (q_k - MBSys.qRef));
    end

    % Normalize Potential energy: Set initial value to 0
    U = U - U(1);

    % Assign to object
    simEnergies = MBSimEnergies;
    simEnergies.H = T + U + V;
    simEnergies.T = T;
    simEnergies.U = U;
    simEnergies.V = V;
end