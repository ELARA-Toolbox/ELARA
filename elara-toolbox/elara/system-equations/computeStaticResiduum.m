function res = computeStaticResiduum(MBSys, simPars, q, u)
    %% Compute Residuum of Static Equilbrium Equation
    arguments
        % Multibody system
        MBSys   (1,1) elara.SystemNum

        simPars (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF)
        q       (:,1) double

        % Vector of inputs (1,nInputs)
        u       (:,1) double
    end

    % Check argument sizes
    assert( size(q,1) == MBSys.nDoF, ...
        "Vector of generalized coordinates has wrong dimensions.");
    assert( size(u,1) == MBSys.nInputs, ...
        "Vector of inputs has wrong dimensions.");

    % Forward Kinematics and Jacobians
    [g, g_rel] = MBSys.computeFwdKin(q);
    J = MBSys.computeGeomJacobianFast(q, g_rel);

    % Generalized forces (stress and inputs)
    f_gen = MBSys.cSys .* (q - MBSys.qRef) - MBSys.computeInputMatrixFast(g_rel) * u;

    % Placeholder values for external forces
    f_frame_b = zeros(6, MBSys.nFrames);
    f_frame_s = zeros(6, MBSys.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b = -f_frame_b + computeBodyfixedFrameForces(g, f_frame_s, MBSys, simPars);

    % Compute sum of generalized forces
    res = f_gen;
    for iFrm = 1:MBSys.nFrames
        res = res + J(:,:,iFrm).' * f_frame_b(:,iFrm);
    end

end
