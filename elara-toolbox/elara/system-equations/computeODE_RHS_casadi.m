function [q_ddot] = computeODE_RHS_casadi(q, q_dot, u, MBSys, simPars) %#codegen
    %% Compute the Right-Hand Side of the EOMs in first-order form (with inverted Mass Matrix)
    arguments (Input)
        % Coordinates and coord. Velocities (2*nDof,1)
        q       (:,1)
        q_dot   (:,1)

        % Input vector
        u     (:,1)

        MBSys   (1,1) MBSystemSym

        simPars (1,1) MBSimPars
    end
    arguments (Output)
        q_ddot        (:,1)
    end

    f = getSE3Functions(x);

    %% Relative Kinematics

    % Forward Kinematics and Jacobians
    [g, g_rel] = MBSys.computeFwdKin(q);
    J = MBSys.computeGeomJacobianFast(q, g_rel);

    % Compute absolute velocities
    eta = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        for iBlock = 1:MBSys.nFrames
            qIndices = double( MBSys.frameData.qIndices(1,iBlock):MBSys.frameData.qIndices(2,iBlock));
            if ~isempty(J{iFrm, iBlock})
                if isempty(eta{iFrm})
                    eta{iFrm} = J{iFrm, iBlock} * q_dot(qIndices);
                else
                    eta{iFrm} = eta{iFrm} + J{iFrm, iBlock} * q_dot(qIndices);
                end
            end
        end
    end
    J_dot = MBSys.computeGeomJacobianTimeDerivativeFast(q, q_dot, eta, g_rel);

    %% Evaluate EOM

    % Generalized forces (stress, dissipation and system inputs)
    f_gen = MBSys.cSys .* (q - MBSys.qRef) ...
        + MBSys.dSys .* q_dot;

    % Input term
    f_gen_C = cell(MBSys.nFrames, 1);
    B = MBSys.computeInputMatrix(q);
    for iFrm = 1:MBSys.nFrames
        for iInput = 1:MBSys.nInputs
            if ~isempty(B{iFrm, iInput})
                if isempty(f_gen_C{iFrm})
                    f_gen_C{iFrm} = -B{iFrm, iInput} * u(iInput);
                else
                    f_gen_C{iFrm} = f_gen_C{iFrm} - B{iFrm, iInput} * u(iInput);
                end
            end
        end
    end

    % Placeholder values for external forces
    f_frame_b = zeros(6, MBSys.nFrames);
    f_frame_s = zeros(6, MBSys.nFrames);

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_b_C = computeBodyFixedFrameForces_sym(MBSys, g, f_frame_s, simPars.g);

    % Compute J_dot * eta
    JdotTerm = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        for iBlock = 1:MBSys.nFrames
            qIndices = double( MBSys.frameData.qIndices(1,iBlock):MBSys.frameData.qIndices(2,iBlock));
            if ~isempty(J_dot{iFrm, iBlock})
                if isempty(JdotTerm{iFrm})
                    JdotTerm{iFrm} = J_dot{iFrm, iBlock} * q_dot(qIndices);
                else
                    JdotTerm{iFrm} = JdotTerm{iFrm} + J_dot{iFrm, iBlock} * q_dot(qIndices);
                end
            end
        end
    end

    for iFrm = 1:MBSys.nFrames
        % Overall frame forces
        f_frame_b_i = ...
            - f_frame_b(:, iFrm) ...
            + f_frame_b_C{iFrm} ...
            ...% Coriolis Term
            + MBSys.frameData.MGen(:,:,iFrm) * JdotTerm{iFrm} ...
            - f.sadSE3(eta{iFrm}(1:3),eta{iFrm}(4:6)).' * MBSys.frameData.MGen(:,:,iFrm) * eta{iFrm};
        % Distribute node terms to coordinates
        for iB = 1:MBSys.nFrames
            if ~isempty(J{iFrm,iB})
                if isempty(f_gen_C{iB})
                    f_gen_C{iB} = J{iFrm,iB}.' * f_frame_b_i;
                else
                    f_gen_C{iB} = f_gen_C{iB} + J{iFrm,iB}.' * f_frame_b_i;
                end
            end
        end
    end

    f_gen = vertcat(f_gen_C{:}) + f_gen;

    %% Assemble first-order RHS
    M_C = MBSys.computeMassMatrix(q);
    M_rows = cell(MBSys.nFrames,1);
    for iFrm = 1:MBSys.nFrames
        M_rows{iFrm} = horzcat(M_C{iFrm,:});
    end
    M = vertcat(M_rows{:});
    q_ddot = -M\f_gen;
end