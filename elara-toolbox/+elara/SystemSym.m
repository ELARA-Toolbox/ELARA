classdef SystemSym < elara.abstract.System
    %% Symbolic representation of an ELARA multibody system
    % Specifies a complete multibody system in tree topology consisting
    % of several rigid or flexible links.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %% Data of the individual frames
        frames   (1,1) elara.FramePropertiesSym

        %% Global system properties

        % Vector of stiffness coefficients for all coordinates
        % (i.e., diagonal entries of the generalized stiffness matrix)
        cSys        (:,1)

        % Vector of (linear) dissipation coefficients for all coordinates
        dSys        (:,1)

        % Vector of reference deformations written in generalized
        % coordinates form
        qRef        (:,1)
    end

    methods
        function q = setJointAngles(system,theta)
            %% Return coordinate vector with specified joint angles
            % The rest of q is zero.
            arguments
                system  (1,1) elara.SystemSym

                % Vector of joint angles with dimension (nJoints,1)
                theta      (:,1)
            end
            assert(size(theta,1) == system.nJoints, "Joint angle vector has incorrect length.");

            % Get indices of frames with screw joints
            thetaIndices = system.frames.qIndices(1,system.frames.jointType == 1);

            % Set coordinates
            q = zeros(system.nDoF,1);
            q(thetaIndices) = theta;
        end
        function q = setLinkDeformations(system, xi, iLink)
            %% Return coordinate vector with specified beam deformations
            arguments
                system  (1,1) elara.SystemSym

                % Array of discrete deformations with size (6,nSegments) for
                % the current link
                xi      (:,:)

                % Index of the current link
                iLink   (1,1) double
            end

            % Get frames belonging to the link
            if iLink == 1 && system.isCantilever
                % Cantilever link: All frames correspond to beam segments
                linkFrames = system.linkFrameIndices(1, iLink):system.linkFrameIndices(2, iLink);
            else
                % Regular link: First frame has screw joint and does not
                % correspond to a beam segment
                linkFrames = (system.linkFrameIndices(1, iLink)+1):system.linkFrameIndices(2, iLink);
            end

            nSegments = numel(linkFrames);
            assert(size(xi,2) == nSegments, ...
                "The deformation matrix xi must have one column per beam segment.")

            % Get indices in q belonging to the flexible link
            % Note: We assume all coordinates of the link are stored
            %       consecutively in the coordinate vector q
            qIndices = system.frames.qIndices(1,linkFrames(1)):system.frames.qIndices(2,linkFrames(end));

            % Store coordinates in q
            % Note: We assume all segments have the same Ba matrix
            Ba = system.frames.Ba(linkFrames(1));
            psi = Ba.' * xi;
            q = zeros(system.nDoF,1);
            q(qIndices) = psi(:);
        end

        function theta = getJointAngles(system, q)
            %% Return vector of joint angles for given coordinate vector
            arguments
                system  (1,1) elara.SystemSym

                % Vector of generalized coordinates from which the joint
                % angles should be returned
                q      (:,1)
            end
            % Get indices of frames with screw joints
            thetaIndices = system.frames.qIndices(1,system.frames.jointType == 1);

            % Get angles
            theta = q(thetaIndices);
        end

        function xi = getLinkDeformations(system, q, iLink)
            %% Return array of discrete deformations for given link and coordinate vector
            arguments
                system (1,1) elara.SystemSym

                % Vector of generalized coordinates from which the
                % discrete deformations shall be computed
                q      (:,1)

                % Link for which to return the discrete deformations
                iLink  (1,1) double
            end

            % Get frames belonging to the link
            if iLink == 1 && system.isCantilever
                % Cantilever link: All frames correspond to beam segments
                linkFrames = system.linkFrameIndices(1, iLink):system.linkFrameIndices(2, iLink);
            else
                % Regular link: First frame has screw joint and does not
                % correspond to a beam segment
                linkFrames = (system.linkFrameIndices(1, iLink)+1):system.linkFrameIndices(2, iLink);
            end

            nSegments = numel(linkFrames);

            % Get indices in q belonging to the flexible link
            % Note: We assume all coordinates of the link are stored
            %       consecutively in the coordinate vector q
            qIndices = double( ...
                system.frames.qIndices(1,linkFrames(1)):system.frames.qIndices(2,linkFrames(end)) ...
                );

            % Get coordinates and store them in array of size
            % (nAllwd,nSegments)
            % Note: We assume all segments have the same number of allowed
            %       deformation modes.
            nAllwd = system.frames.nDof(linkFrames(1));
            psi = reshape(q(qIndices), nAllwd, nSegments);

            % Compute complete array of deformations
            % Note: We assume all segments have the same Ba matrix
            Ba = system.frames.Ba(linkFrames(1));
            xi = Ba * psi + system.frames.xiC(:,linkFrames);
        end

        function g_rel = computeJointTransformations(system,q)
            %% Compute the relative transformations of all joints (rigid and flexible)
            % for given relative coordinates
            arguments (Input)
                system  (1,1) elara.SystemSym

                % System coordinates  (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % SE(3) elements with relative configurations between body frames
                g_rel   (:,1) elara.SE3.Element
            end
            f = elara.internal.math.getSE3Functions(q);
            g_rel = createArray(system.nFrames,1, "elara.SE3.Element");
            for iFrm = 1:system.nFrames
                % CasADi does not accept uint16 values as indices.
                indices = double(system.frames.qIndices(1,iFrm):system.frames.qIndices(2,iFrm));
                qi = q(indices);
                switch system.frames.jointType(iFrm)
                    case 1
                        %%% Screw joint
                        g_screw = elara.SE3.Element;
                        g_ref = elara.SE3.Element(system.frames.g_ref(1:3,1:3,iFrm),system.frames.g_ref(1:3,4,iFrm));
                        [g_screw.R, g_screw.x] = f.SE3.expScrew(system.frames.X(1:3,iFrm), system.frames.X(4:6,iFrm), qi);
                        g_rel(iFrm) = g_ref * g_screw;
                    case 2
                        %%% Flexible joint
                        Ba = system.frames.Ba(iFrm);
                        om = (Ba(1:3,:) * qi + system.frames.xiC(1:3,iFrm));
                        v  = (Ba(4:6,:) * qi + system.frames.xiC(4:6,iFrm));
                        [g_rel(iFrm).R, g_rel(iFrm).x] = f.SE3.cay(om*system.frames.l(iFrm), v*system.frames.l(iFrm));
                    otherwise
                        error("Invalid joint type specified.");
                end
            end
        end

        function g = computeFwdKinFast(system, g_rel)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            % with *given* relative joint transformations g_ij
            arguments (Input)
                system  (1,1) elara.SystemSym

                % Array of relative configurations between body frames
                % dimensions (nFrames, 1)
                g_rel   (:,1) elara.SE3.Element
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (:,1) elara.SE3.Element
            end

            %% Compute kinematics
            % Kinematics without Joint Frames, Section 2.3 (CoM Frames = Body Frames only)
            g = createArray(system.nFrames,1, "elara.SE3.Element");

            % First frame
            g0 = elara.SE3.Element(system.g0(1:3,1:3), system.g0(1:3,4));
            g(1) = g0 * g_rel(1);

            % Other frames
            for iFrm = 2:system.nFrames
                g(iFrm) = g(system.frames.parent(iFrm)) * g_rel(iFrm);
            end
        end

        function [g, g_rel] = computeFwdKin(system, q)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            arguments (Input)
                system  (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (:,1) elara.SE3.Element

                % Relative configurations between body frames (joint
                % transformations)
                g_rel   (:,1) elara.SE3.Element
            end
            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute kinematics
            g = computeFwdKinFast(system, g_rel);
        end

        function J = computeGeomJacobianFast(system, q, g_rel)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            % with *given* relative joint transformations g_ij
            arguments (Input)
                system  (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel   (:,1) elara.SE3.Element
            end
            arguments (Output)
                % Cell array with dimensions nFrames x nFrames
                % Each row contains the individual blocks of the full
                % Jacobian matrix corresponding to the frame
                J       (:,:) cell
            end
            f = elara.internal.math.getSE3Functions(q);

            % Array holding all Jacobians
            J = cell(system.nFrames, system.nFrames);
            for iFrm = 1:system.nFrames
                for iBlock = 1:iFrm
                    % Column indices of the current block
                    qIndices = double(system.frames.getQIndices(iBlock));

                    % Compute block columns for current frame
                    if iBlock == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                J{iFrm, iBlock} = system.frames.X(:,iFrm);
                            case 2
                                Ba = system.frames.Ba(iFrm);
                                om = (Ba(1:3,:) * q(qIndices) + system.frames.xiC(1:3,iFrm))*system.frames.l(iFrm);
                                v  = (Ba(4:6,:) * q(qIndices) + system.frames.xiC(4:6,iFrm))*system.frames.l(iFrm);
                                J{iFrm, iBlock} = ...
                                    system.frames.l(iFrm) * f.SE3.dcay(-om, -v) ...
                                    * system.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if iBlock < iFrm && ismember(iBlock, system.frames.ancestors(:,iFrm))
                            J{iFrm, iBlock} = f.SE3.AdInv(g_rel(iFrm).R, g_rel(iFrm).x) ...
                                * J{system.frames.parent(iFrm),iBlock};
                        end
                    end
                end
            end
        end

        function [J, g_rel] = computeGeomJacobian(system, q)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            arguments (Input)
                system  (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x (nAllwd_1*nSegments1 + ... + nAllwdB*nSegmentsB + nLinks) x nFrames
                % where B is the number of flexible beams in the system
                J       (:,:) cell

                % Relative configurations between body frames
                g_rel   (:,1) elara.SE3.Element
            end
            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute Jacobians
            J = system.computeGeomJacobianFast(q, g_rel);
        end

        function J_bias = computeGeomJacobianAccelerationBiasMatrixFast(system, q, q_dot, eta, g_rel)
            %% Compute a Geometric-Jacobian Acceleration-Bias Matrix
            % This matrix is not the true time derivative of the geometric
            % Jacobian. Contracting each block row with the corresponding
            % blocks of q_dot gives the same result as J_dot*q_dot, where
            % J_dot denotes the true derivative. The equations of motion
            % require only this contracted acceleration-bias term.
            % "Fast" function - with given relative transformations g_ij
            arguments (Input)
                system      (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)

                % Absolute frame velocities
                eta         (:,1) cell

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel       (:,1) elara.SE3.Element
            end
            arguments (Output)
                % Acceleration-bias factorization blocks
                J_bias       (:,:) cell
            end

            %%%% TODO:
            %%%% Decide if we keep eta as six-dimensional or use separate
            %%%% omega, v

            f = elara.internal.math.getSE3Functions(q);
            fSE3DcayDerivative = elara.internal.math.getCayRTDSE3dtFunction(q);

            % Array holding all acceleration-bias matrices
            J_bias = cell(system.nFrames, system.nFrames);
            for iFrm = 1:system.nFrames
                AdInvRel = f.SE3.AdInv(g_rel(iFrm).R, g_rel(iFrm).x);
                for iBlock = 1:iFrm
                    % Column indices of the current block
                    qIndices = double(system.frames.getQIndices(iBlock));

                    % Compute block columns for current frame
                    if iBlock == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                J_bias{iFrm, iBlock} = ...
                                    f.SE3.smallAd(eta{iBlock}(1:3),eta{iBlock}(4:6)) * system.frames.X(:,iFrm);
                            case 2
                                l = system.frames.l(iFrm);
                                Ba = system.frames.Ba(iFrm);
                                om = (Ba(1:3,:) * q(qIndices) + system.frames.xiC(1:3,iFrm))*l;
                                v  = (Ba(4:6,:) * q(qIndices) + system.frames.xiC(4:6,iFrm))*l;
                                om_dot = Ba(1:3,:) * q_dot(qIndices)*l;
                                v_dot  = Ba(4:6,:) * q_dot(qIndices)*l;

                                J_bias{iFrm, iBlock} = l * ( ...
                                    f.SE3.smallAd(eta{iBlock}(1:3), eta{iBlock}(4:6)) * f.SE3.dcay(-om, -v) ...
                                    + fSE3DcayDerivative(-om, -v, -om_dot, -v_dot ) ...
                                    ) * system.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ismember(iBlock, system.frames.ancestors(:,iFrm))
                            J_bias{iFrm, iBlock} = AdInvRel ...
                                * J_bias{system.frames.parent(iFrm),iBlock};
                        end
                    end
                end
            end
        end

        function [J_bias, g_rel] = computeGeomJacobianAccelerationBiasMatrix(system, q, q_dot, eta)
            %% Compute a Geometric-Jacobian Acceleration-Bias Matrix
            % This is not the true Jacobian derivative. It is an efficient
            % factorization satisfying J_bias*q_dot = J_dot*q_dot and is
            % therefore used by the equations of motion.
            arguments
                system      (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)

                % Absolute frame velocities
                eta         (:,1) cell
            end

            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute acceleration-bias factorization
            J_bias = computeGeomJacobianAccelerationBiasMatrixFast( ...
                system, q, q_dot, eta, g_rel);
        end

        function J_dot = computeGeomJacobianTimeDerivativeFast(system, q, q_dot, J, g_rel)
            %% Compute the True Time Derivative of the Geometric Jacobian
            % Unlike computeGeomJacobianAccelerationBiasMatrixFast, this
            % method differentiates every Jacobian block. Use the bias
            % matrix when only J_dot*q_dot is required.
            % "Fast" function - with given Jacobians and relative
            % transformations g_ij
            arguments (Input)
                system      (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)

                % Geometric Jacobian blocks
                J           (:,:) cell

                % Array of relative configurations between body frames
                g_rel       (:,1) elara.SE3.Element
            end
            arguments (Output)
                % True derivative of the geometric Jacobian blocks
                J_dot       (:,:) cell
            end

            f = elara.internal.math.getSE3Functions(q);
            fSE3DcayDerivative = elara.internal.math.getCayRTDSE3dtFunction(q);

            J_dot = cell(system.nFrames, system.nFrames);
            for iFrm = 1:system.nFrames
                currentQIndices = double(system.frames.getQIndices(iFrm));
                eta_rel = J{iFrm,iFrm} * q_dot(currentQIndices);
                AdInvRel = f.SE3.AdInv(g_rel(iFrm).R, g_rel(iFrm).x);

                for iBlock = 1:iFrm
                    qIndices = double(system.frames.getQIndices(iBlock));

                    if iBlock == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                % The local screw-joint block is constant.
                                J_dot{iFrm,iBlock} = zeros(6, numel(qIndices));
                            case 2
                                l = system.frames.l(iFrm);
                                Ba = system.frames.Ba(iFrm);
                                om = (Ba(1:3,:) * q(qIndices) ...
                                    + system.frames.xiC(1:3,iFrm)) * l;
                                v = (Ba(4:6,:) * q(qIndices) ...
                                    + system.frames.xiC(4:6,iFrm)) * l;
                                om_dot = Ba(1:3,:) * q_dot(qIndices) * l;
                                v_dot = Ba(4:6,:) * q_dot(qIndices) * l;

                                J_dot{iFrm,iBlock} = l ...
                                    * fSE3DcayDerivative( ...
                                    -om, -v, -om_dot, -v_dot) ...
                                    * Ba;
                            otherwise
                                % error
                        end
                    else
                        if ismember(iBlock, system.frames.ancestors(:,iFrm))
                            % J_i,iBlock = AdInvRel*J_parent,iBlock.
                            % Reuse the child block to avoid an additional
                            % symbolic matrix product.
                            J_dot{iFrm,iBlock} = ...
                                - f.SE3.smallAd(eta_rel(1:3), eta_rel(4:6)) ...
                                * J{iFrm,iBlock} ...
                                + AdInvRel ...
                                * J_dot{system.frames.parent(iFrm),iBlock};
                        end
                    end
                end
            end
        end

        function [J_dot, J, g_rel] = computeGeomJacobianTimeDerivative(system, q, q_dot)
            %% Compute the True Time Derivative of the Geometric Jacobian
            % Use computeGeomJacobianAccelerationBiasMatrix instead when
            % only the contracted term J_dot*q_dot is required.
            arguments
                system      (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)
            end

            g_rel = system.computeJointTransformations(q);
            J = system.computeGeomJacobianFast(q, g_rel);
            J_dot = computeGeomJacobianTimeDerivativeFast( ...
                system, q, q_dot, J, g_rel);
        end

        function B = computeInputMatrixFast(system, g_rel)
            %% Compute the system input matrix
            % "Fast" function -- with given relative deformations
            arguments
                system      (1,1) elara.SystemSym

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel       (:,1) elara.SE3.Element
            end
            f = elara.internal.math.getSE3Functions(g_rel(1).R);

            B = cell(system.nFrames, system.nInputs);

            for iFrm = 1:system.nFrames
                if system.frames.uIndices(1,iFrm)
                    uIndices = double(system.frames.getUIndices(iFrm));
                    switch system.frames.jointType(iFrm)
                        case 1
                            % Rigid joint (scalar input)
                            B{iFrm, uIndices} = 1;
                        case 2
                            % Flexible joint (multiple cable inputs)
                            l = system.frames.l(iFrm);

                            for iC = 1:length(uIndices)
                                if ~system.frames.tendonIsActive(iFrm,iC)
                                    continue;
                                end

                                % Cable configurations at adjacent nodes
                                g_cm_i1 = system.frames.g_cm(1,iFrm,iC);
                                g_cm_i2 = system.frames.g_cm(2,iFrm,iC);

                                % Discrete deformation gradient cable routing
                                % Tangent vector is in elements 4:6
                                g_rel_c = g_cm_i1 \ g_rel(iFrm) * g_cm_i2;
                                [~, v_c] = f.SE3.cayInv(g_rel_c.R, g_rel_c.x);
                                v_c = v_c / l;

                                % Compute matrix entry
                                B{iFrm, uIndices(iC)} = ...
                                    -l / norm(v_c) * system.frames.Ba(iFrm).' * [
                                    1/2 * ( elara.SO3.skew( g_cm_i1.x + g_cm_i2.x ) ) * v_c;
                                    v_c
                                    ];
                            end
                        otherwise
                            % error
                    end
                end
            end
        end

        function B = computeInputMatrix(system, q)
            %% Compute the system input matrix
            arguments
                system      (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)
            end
            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute input matrix
            B = system.computeInputMatrixFast(g_rel);
        end

        function M = computeMassMatrixFast(system, J)
            %% Compute the system mass matrix
            arguments (Input)
                system  (1,1) elara.SystemSym

                % Cell array of geometric Jacobians
                J       (:,:) cell
            end
            arguments (Output)
                % System mass matrix
                M       (:,:) cell
            end
            % Compute blocks of M based on dyadic product J.' * M * J
            M = cell(system.nFrames, system.nFrames);
            for iFrm = 1:system.nFrames % Sum over all the frame contributions
                for iRow = 1:system.nFrames % Rows of M
                    for iCol = 1:system.nFrames % Columns of M
                        if ~isempty(J{iFrm,iRow}) && ~isempty(J{iFrm,iCol})
                            MBlock = J{iFrm,iRow}.' * system.frames.MGen{iFrm} * J{iFrm,iCol};
                            if isempty(M{iRow, iCol})
                                M{iRow,iCol} = MBlock;
                            else
                                M{iRow,iCol} = M{iRow,iCol} + MBlock;
                            end
                        end
                    end
                end
            end
        end

        function [M, J] = computeMassMatrix(system, q)
            %% Compute the system mass matrix
            arguments (Input)
                system  (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % System mass matrix
                M       (:,:) cell

                % Array of frame Jacobian matrices
                J       (:,:) cell
            end

            % Get Jacobians
            J = system.computeGeomJacobian(q);

            % Compute mass matrix
            M = computeMassMatrixFast(system, J);
        end

        function eta_k = computeDiscreteAbsoluteVelocities(system, g_rel_k, g_rel_k1, h)
            %% Compute discrete absolute velocities in the interval (k,k+1)
            % i.e., compute absolute body-fixed velocities eta in se(3)
            % (vector form) from given relative transformations at time
            % instances k and k+1
            arguments (Input)
                system      (1,1) elara.SystemSym

                % Cell array of relative configurations at time step k
                g_rel_k     (:,1) elara.SE3.Element

                % Cell array of relative configurations at time step k+1
                g_rel_k1    (:,1) elara.SE3.Element

                % Time step
                h           (1,1)
            end
            arguments (Output)
                % Array of absolute frame velocities
                % (separate rotational and translational components)
                eta_k       (:,2) cell
            end
            f = elara.internal.math.getSE3Functions(g_rel_k(1).R);

            eta_k = cell(system.nFrames, 2);

            % Recursive computation for all frames
            for iFrm = 1:system.nFrames
                iParent = system.frames.parent(iFrm);
                if iParent > 0
                    g_eta = elara.SE3.Element;
                    [g_eta.R, g_eta.x] = f.SE3.cay( ...
                        h*eta_k{iParent, 1}, h*eta_k{iParent, 2});
                    g_rel = g_rel_k(iFrm) \ g_eta * g_rel_k1(iFrm);
                else
                    g_rel = g_rel_k(iFrm) \ g_rel_k1(iFrm);
                end
                [om_k, v_k] = f.SE3.cayInv(g_rel.R, g_rel.x);
                eta_k{iFrm,1} = om_k / h;
                eta_k{iFrm,2} = v_k / h;
            end
        end
    end
end
