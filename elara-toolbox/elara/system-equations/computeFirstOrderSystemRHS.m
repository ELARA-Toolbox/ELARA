function f_fo = computeFirstOrderSystemRHS(t, x, MBSys, simPars) %#codegen
    %% Compute the Right-Hand Side of the EOMs in first-order form
    arguments (Input)
        % Integration time (from ode solver)
        t           (1,1) double

        % State vector [q; q_dot] (2*nDof,1)
        x           (:,1) double

        MBSys       (1,1) elara.SystemNum

        simPars     (1,1) MBSimPars
    end
    arguments (Output)
        f_fo        (:,1) double
    end

    %% Get configuration and velocity
    q     = x(1:MBSys.nDoF);
    q_dot = x((MBSys.nDoF+1):end);


    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = MBSys.computeFwdKin(q);
    J = MBSys.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    eta = zeros(6,MBSys.nFrames);
    for iFrm = 1:MBSys.nFrames
        eta(:,iFrm) = J(:,:,iFrm) * q_dot;
    end
    J_dot = MBSys.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);

    %% Prepare system inputs
    % Constant inputs
    if ~isempty(simPars.uConst) && size(simPars.uConst,1) == MBSys.nInputs
        u_k = simPars.uConst;
    else
        u_k = zeros(MBSys.nInputs, 1);
    end

    % Time-varying inputs
    if (~isempty(simPars.uSampleValues) && size(simPars.uSampleValues,1) == MBSys.nInputs ) && ...
            ~isempty(simPars.uSampleTimes) && ...
            size(simPars.uSampleValues,2) == size(simPars.uSampleTimes,1)
        u_k = u_k + interp1(simPars.uSampleTimes, simPars.uSampleValues.', t, 'linear', 0).';
    end

    %% Evaluate EOM

    % Generalized forces (stress, dissipation and system inputs)
    f_gen = MBSys.cSys .* (q - MBSys.qRef) ...
        + MBSys.dSys .* q_dot ...
        - MBSys.computeInputMatrix(q) * u_k;

    % External frame forces from the environment
    f_frame_b = getExternalStepWrenches(simPars.extWrench_b, MBSys.nFrames, t);
    f_frame_s = getExternalStepWrenches(simPars.extWrench_s, MBSys.nFrames, t);


    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + computeBodyfixedFrameForces(g, f_frame_s, MBSys, simPars);

    % Compute sum of generalized forces
    for iFrm = 1:MBSys.nFrames
        f_gen = f_gen + J(:,:,iFrm).' * (...
            ...% Frame forces
            + f_frame_b(:,iFrm)...
            ...% Coriolis Term
            + (MBSys.frames.MGen(:,:,iFrm) * J_dot(:,:,iFrm) ...
            - sadSE3(eta(:,iFrm)).' * MBSys.frames.MGen(:,:,iFrm) * J(:,:,iFrm)) * q_dot ...
            );
    end

    %% Assemble first-order RHS
    f_fo = [q_dot; -f_gen];

end