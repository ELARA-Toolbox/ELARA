function [res, g, g_rel] = computeStaticResiduum_casadi(MBSys, simPars, q, u)
    %% Compute Residuum of Static Equilbrium Equation
    arguments
        % Multibody system
        MBSys   (1,1) elara.SystemSym

        simPars (1,1) MBSimPars

        % Vector of generalized coordinates (1,nDoF)
        q       (:,1)

        % Vector of inputs (1,nInputs)
        u       (:,1)
    end

    % Check argument sizes
    assert( size(q,1) == MBSys.nDoF, ...
        "Vector of generalized coordinates has wrong dimensions.");
    assert( size(u,1) == MBSys.nInputs, ...
        "Vector of inputs has wrong dimensions.");

    % Forward Kinematics and Jacobians
    [g, g_rel] = MBSys.computeFwdKin(q);
    J = MBSys.computeGeomJacobianFast(q, g_rel);

    % Input matrix
    B = MBSys.computeInputMatrixFast(g_rel);

    % Generalized forces (stress)
    f_gen = MBSys.cSys .* (q - MBSys.qRef);

    % Placeholder values for external forces
    f_frame_b = zeros(6, MBSys.nFrames);
    f_frame_s = zeros(6, MBSys.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    %f_frame_b = -f_frame_b + computeBodyfixedFrameForces(g, f_frame_s, MBSys, simPars);
    f_frame_b_C = computeBodyfixedFrameForces_sym(MBSys, g, f_frame_s, simPars.g);

    % Compute sum of generalized forces
    resC = cell(MBSys.nFrames, 1);
    for iFrm = 1:MBSys.nFrames
        % Overall frame forces
        f_frame_b_i = - f_frame_b(:, iFrm) + f_frame_b_C{iFrm};

        % Distribute node terms to coordinates
        for iB = 1:MBSys.nFrames
            if ~isempty(J{iFrm,iB})
                if isempty(resC{iB})
                    resC{iB} = J{iFrm,iB}.' * f_frame_b_i;
                else
                    resC{iB} = resC{iB} + J{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    % Add input terms to coordinates
    for iFrm = 1:MBSys.nFrames
        for iInput = 1:MBSys.nInputs
            if ~isempty(B{iFrm, iInput})
                resC{iFrm} = resC{iFrm} - B{iFrm, iInput} * u(iInput);
            end
        end
    end

    res = vertcat(resC{:}) + f_gen;

    %x = {g.x};
end