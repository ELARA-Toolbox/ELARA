function [T,U,V,H] = computeEnergies(system, simPars, simRes, isVarInt, useFiniteDifferences) %#codegen
    %% Compute energy evolution for simulation results
    arguments (Input)
        system      (1,1) elara.SystemNum
        simPars     (1,1) elara.SimulationParameters
        simRes      (1,1) elara.SimulationResults

        % Defines whether simulation results belong to a variational
        % integrator
        isVarInt    (1,1) logical

        % For varInts: Whether to compute the kinetic energy using generalized
        % velocities computed from finite differences (true) or via
        % discrete momenta
        useFiniteDifferences (1,1) logical
    end

    nSteps = numel(simRes.tout);
    T = zeros(nSteps, 1);
    U = zeros(nSteps, 1);
    V = zeros(nSteps, 1);

    % Trapezoidal rule factor
    a = 1/2;

    % Get system inputs for varInts
    if isVarInt
        u = elara.internal.simulation.evaluateSystemInputs(system, simPars, simRes.tout);
        h = simRes.tout(2) - simRes.tout(1);

        % Get velocities via finite differences
        if useFiniteDifferences
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
        if isVarInt && ~useFiniteDifferences
            % Variational integrators: Compute kinetic energy at
            % time nodes based on discrete momenta
            if iStep < nSteps
                % Steps 1, ..., N-1: Left momentum
                q_k1 = simRes.q(:,iStep+1);
                p_k = elara.dynamics.num.leftGeneralizedMomentum(system, h, simPars, ...
                    q_k, q_k1, g_k, simRes.eta(:,:,iStep), u(:,iStep), a);
            else
                % Last step: Right momentum
                q_k0 = simRes.q(:,iStep-1);
                p_k = elara.dynamics.num.rightGeneralizedMomentum(system, h, simPars, ...
                    q_k0, q_k, g_k, simRes.eta(:,:,iStep-1), u(:,iStep), a);
            end
            T(iStep) = 1/2 * p_k.' / system.computeMassMatrix(q_k) * p_k;
        else
            % ODE integrators: Directly compute kinetic energy from
            % time node velocities
            q_dot_k = q_dot(:,iStep);
            T(iStep) = 1/2 * q_dot_k.' * system.computeMassMatrix(q_k) * q_dot_k;
        end

        for iFrm = 1:system.nFrames
            % Potential energy (both frame CoM and attached masses)
            g_a_k = g_k(:,:,iFrm) * system.frames.g_a(:,:,iFrm);
            z_k = g_k(3,4,iFrm);

            % The attached mass is already included in frames.m. Correct
            % its contribution from the frame origin to its actual CoM.
            U(iStep) = U(iStep) + simPars.g *(...
                system.frames.m(iFrm) * z_k + ...
                system.frames.m_a(iFrm) * (g_a_k(3,4) - z_k)...
                );
        end

        % Strain energy
        V(iStep) = 1/2 * (q_k - system.qRef).' ...
            * (system.cSys .* (q_k - system.qRef));
    end

    % Normalize Potential energy: Set initial value to 0
    U = U - U(1);

    % Total energy 
    H = T + U + V;
end
