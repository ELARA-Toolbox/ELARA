classdef SystemNum < elara.abstract.System
    %% Numeric representation of an ELARA multibody system
    % Specifies a complete multibody system in tree topology consisting
    % of several rigid or flexible links.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %% Data of the individual frames
        frames   (1,1) elara.FramePropertiesNum

        %% Global system properties

        % Vector of stiffness coefficients for all coordinates
        % (i.e., diagonal entries of the generalized stiffness matrix)
        cSys        (:,1) double

        % Vector of (linear) dissipation coefficients for all coordinates
        dSys        (:,1) double {mustBeNonnegative}

        % Vector of reference deformations written in generalized
        % coordinates form
        qRef        (:,1) double
    end

    methods
        function q = setJointAngles(system,theta)
            %% Return coordinate vector with specified joint angles
            % The rest of q is zero.
            arguments
                system    (1,1) elara.SystemNum

                % Joint angles with dimensions (nJoints, 1)
                theta     (:,1) double
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
                system  (1,1) elara.SystemNum

                % Array of discrete deformations with size (6,nSegments) for
                % the current link
                xi      (:,:) double

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
                system  (1,1) elara.SystemNum

                % Vector of generalized coordinates from which the joint
                % angles should be returned
                q      (:,1) double
            end
            % Get indices of frames with screw joints
            thetaIndices = system.frames.qIndices(1,system.frames.jointType == 1);

            % Get angles
            theta = q(thetaIndices);
        end

        function xi = getLinkDeformations(system, q, iLink)
            %% Return array of discrete deformations for given link and corrdinate vector
            arguments
                system  (1,1) elara.SystemNum

                % Vector of generalized coordinates from which the
                % discrete deformations shall be computed
                q      (:,1) double

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
            qIndices = system.frames.qIndices(1,linkFrames(1)):system.frames.qIndices(2,linkFrames(end));

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
                system     (1,1) elara.SystemNum

                % System coordinates  (nDoF, 1)
                q       (:,1) double
            end
            arguments (Output)
                % SE(3) matrices with relative configurations between body frames
                g_rel   (4,4,:) double
            end
            g_rel = zeros(4,4,system.nFrames);
            for iFrm = 1:system.nFrames
                qi = q(system.frames.qIndices(1,iFrm):system.frames.qIndices(2,iFrm));
                switch system.frames.jointType(iFrm)
                    case 1
                        %%% Screw joint
                        g_rel(:,:,iFrm) = system.frames.g_ref(:,:,iFrm) * elara.SE3.expScrew(system.frames.X(:,iFrm), qi);
                    case 2
                        %%% Flexible joint
                        xi = system.frames.Ba(iFrm) * qi + system.frames.xiC(:,iFrm);
                        g_rel(:,:,iFrm) = elara.SE3.cay(xi*system.frames.l(iFrm));
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
                system  (1,1) elara.SystemNum

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel   (4,4,:) double
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (4,4,:) double
            end

            %% Compute kinematics
            % Kinematics without Joint Frames, Section 2.3 (CoM Frames = Body Frames only)
            g = zeros(4,4,system.nFrames);

            % First frame
            g(:,:,1) = system.g0 * g_rel(:,:,1);

            % Other frames
            for iFrm = 2:system.nFrames
                g(:,:,iFrm) = g(:,:,system.frames.parent(iFrm)) * g_rel(:,:,iFrm);
            end
        end

        function [g, g_rel] = computeFwdKin(system, q)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            arguments (Input)
                system  (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q       (:,1) double
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (4,4,:) double

                % Relative configurations between body frames (joint
                % transformations)
                g_rel   (4,4,:) double
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
                system  (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q       (:,1) double

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel   (4,4,:) double
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x nDoF x nFrames
                % where B is the number of flexible beams in the system
                J       (6,:,:) double
            end

            % Array holding all Jacobians
            J = zeros(6, system.nDoF, system.nFrames);
            for iFrm = 1:system.nFrames
                for ii = 1:iFrm
                    % Column indices of the current block
                    qIndices = system.frames.getQIndices(ii);

                    % Compute block columns for current frame
                    if ii == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                J(:,qIndices,iFrm) = system.frames.X(:,iFrm);
                            case 2
                                xi = system.frames.Ba(iFrm) * q(qIndices) + system.frames.xiC(:,iFrm);
                                J(:,qIndices,iFrm) = ...
                                    system.frames.l(iFrm) * elara.SE3.dcay( -xi * system.frames.l(iFrm) ) ...
                                    * system.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ii < iFrm && ismember(ii, system.frames.ancestors(:,iFrm))
                            J(:,qIndices,iFrm) = elara.SE3.AdInv( g_rel(:,:,iFrm) ) ...
                                * J(:,qIndices,system.frames.parent(iFrm));
                        end
                    end
                end
            end
        end

        function [J, g_rel] = computeGeomJacobian(system, q)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            arguments (Input)
                system  (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q       (:,1) double
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x nDoF x nFrames
                J       (6,:,:) double

                % Relative configurations between body frames
                g_rel    (4,4,:) double
            end
            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute Jacobians
            J = system.computeGeomJacobianFast(q, g_rel);
        end

        function J_bias = computeGeomJacobianAccelerationBiasMatrixFast(system, q, q_dot, eta, g_rel)
            %% Compute a Geometric-Jacobian Acceleration-Bias Matrix
            % This matrix is not the true time derivative of the geometric
            % Jacobian. It is an efficient factorization satisfying
            % J_bias(:,:,iFrm)*q_dot = J_dot(:,:,iFrm)*q_dot, where J_dot
            % denotes the true derivative. The equations of motion require
            % only this contracted acceleration-bias term.
            % "Fast" function - with given relative transformations g_ij
            arguments (Input)
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1) double

                % Absolute frame velocities
                eta         (6,:) double

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel        (4,4,:) double
            end
            arguments (Output)
                % Acceleration-bias factorization matrix
                J_bias      (6,:,:) double
            end

            % Array holding all acceleration-bias matrices
            J_bias = zeros(6, system.nDoF, system.nFrames);
            for iFrm = 1:system.nFrames
                AdInvRel = elara.SE3.AdInv(g_rel(:,:,iFrm));
                for ii = 1:iFrm
                    % Column indices of the current block
                    qIndices = system.frames.getQIndices(ii);

                    % Compute block columns for current frame
                    if ii == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                J_bias(:,qIndices,iFrm) = ...
                                    elara.SE3.smallAd(eta(:,ii)) * system.frames.X(:,iFrm);
                            case 2
                                xi     = system.frames.Ba(iFrm) * q(qIndices) + system.frames.xiC(:,iFrm);
                                xi_dot = system.frames.Ba(iFrm) * q_dot(qIndices);
                                l = system.frames.l(iFrm);
                                J_bias(:,qIndices,iFrm) = l * ( ...
                                    + elara.SE3.smallAd(eta(:,ii)) * elara.SE3.dcay(-xi*l) ...
                                    + elara.SE3.dcayDerivative(-xi*l, -xi_dot*l ) ...
                                    ) * system.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ismember(ii, system.frames.ancestors(:,iFrm))
                            J_bias(:,qIndices,iFrm) = AdInvRel ...
                                * J_bias(:,qIndices,system.frames.parent(iFrm));
                        end
                    end
                end
            end
        end
        function J_dot = computeGeomJacobianTimeDerivativeFast(system, q, q_dot, J, g_rel)
            %% Compute the True Time Derivative of the Geometric Jacobian
            % Unlike computeGeomJacobianAccelerationBiasMatrixFast, this
            % method differentiates every Jacobian block. Use the bias
            % matrix when only J_dot*q_dot is required.
            % "Fast" function - with given relative transformations g_ij
            arguments (Input)
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1) double

                % Geometric Jacobian
                J           (6,:,:) double

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel        (4,4,:) double
            end
            arguments (Output)
                % Derivative of Jacobian matrix
                J_dot       (6,:,:) double
            end

            % Array holding all Jacobians
            J_dot = zeros(6, system.nDoF, system.nFrames);
            for iFrm = 1:system.nFrames
                % Body velocity of the current frame relative to its parent.
                % This determines the derivative of AdInv(g_rel(:,:,iFrm))
                % for every Jacobian block inherited from the parent.
                currentQIndices = system.frames.getQIndices(iFrm);
                eta_rel = J(:,currentQIndices,iFrm) * q_dot(currentQIndices);
                AdInvRel = elara.SE3.AdInv(g_rel(:,:,iFrm));

                for ii = 1:iFrm
                    % Column indices of the current block
                    qIndices = system.frames.getQIndices(ii);

                    % Compute block columns for current frame
                    if ii == iFrm
                        switch system.frames.jointType(iFrm)
                            case 1
                                % The local screw-joint block is constant.
                            case 2
                                xi     = system.frames.Ba(iFrm) * q(qIndices) + system.frames.xiC(:,iFrm);
                                xi_dot = system.frames.Ba(iFrm) * q_dot(qIndices);
                                l = system.frames.l(iFrm);
                                J_dot(:,qIndices,iFrm) = l ...
                                    * elara.SE3.dcayDerivative(-xi*l, -xi_dot*l) ...
                                    * system.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ismember(ii, system.frames.ancestors(:,iFrm))
                            % J_i,ii = AdInvRel*J_parent,ii. Reusing the
                            % already computed child block avoids an extra
                            % matrix product in the differentiated formula.
                            J_dot(:,qIndices,iFrm) = ...
                                - elara.SE3.smallAd(eta_rel) * J(:,qIndices,iFrm) ...
                                + AdInvRel * J_dot(:,qIndices,system.frames.parent(iFrm));
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
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1) double

                % Absolute frame velocities
                eta         (6,:) double
            end

            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute acceleration-bias factorization
            J_bias = computeGeomJacobianAccelerationBiasMatrixFast( ...
                system, q, q_dot, eta, g_rel);
        end
        function [J_dot, J, g_rel] = computeGeomJacobianTimeDerivative(system, q, q_dot)
            %% Compute the True Time Derivative of the Geometric Jacobian
            % Use computeGeomJacobianAccelerationBiasMatrix instead when
            % only the contracted term J_dot*q_dot is required.
            arguments
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1) double
            end

            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            J = system.computeGeomJacobianFast(q, g_rel);

            % Compute true Jacobian derivative
            J_dot = computeGeomJacobianTimeDerivativeFast(system, q, q_dot, J, g_rel);
        end
        function B = computeInputMatrixFast(system, g_rel)
            %% Compute the system input matrix
            % "Fast" function -- with given relative deformations
            arguments
                system       (1,1) elara.SystemNum

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel        (4,4,:) double
            end

            B = zeros(system.nDoF, system.nInputs);
            for iFrm = 1:system.nFrames
                if system.frames.uIndices(1,iFrm)
                    uIndices = system.frames.getUIndices(iFrm);
                    qIndices = system.frames.getQIndices(iFrm);

                    switch system.frames.jointType(iFrm)
                        case 1
                            % Rigid joint (scalar input)
                            B(qIndices, uIndices) = 1;
                        case 2
                            % Flexible joint (multiple cable inputs)
                            l = system.frames.l(iFrm);

                            for iC = 1:length(uIndices)
                                if ~system.frames.tendonIsActive(iFrm,iC)
                                    continue;
                                end

                                % Cable configurations at adjacent nodes
                                g_cm_i1 = system.frames.g_cm(:,:,1,iFrm,iC);
                                g_cm_i2 = system.frames.g_cm(:,:,2,iFrm,iC);

                                % Discrete deformation gradient cable routing
                                % Tangent vector is in elements 4:6
                                xi_c = elara.SE3.cayInv( g_cm_i1 \ g_rel(:,:,iFrm) * g_cm_i2 ) / l;

                                % Compute matrix entry
                                b_i = system.frames.Ba(iFrm).' * [
                                    1/2 * ( elara.SO3.skew( g_cm_i1(1:3,4) + g_cm_i2(1:3,4) ) ) * xi_c(4:6);
                                    xi_c(4:6)
                                    ];
                                B(qIndices, uIndices(iC)) = -l / norm(xi_c(4:6)) * b_i;
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
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double
            end
            % Compute relative joint transformations
            g_rel = system.computeJointTransformations(q);

            % Compute input matrix
            B = system.computeInputMatrixFast(g_rel);
        end

        function M = computeMassMatrixFast(system, J)
            %% Compute the system mass matrix
            arguments (Input)
                system  (1,1) elara.SystemNum

                % Array of geometric Jacobians
                J       (6,:,:) double
            end
            arguments (Output)
                % System mass matrix
                M   (:,:) double
            end
            M = zeros(system.nDoF);
            for iFrm = 1:system.nFrames
                M = M + J(:,:,iFrm).' * system.frames.MGen(:,:,iFrm) * J(:,:,iFrm);
            end
        end

        function [M, J] = computeMassMatrix(system, q)
            %% Compute the system mass matrix
            arguments (Input)
                system      (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double
            end
            arguments (Output)
                % System mass matrix
                M   (:,:) double

                % Array of frame Jacobian matrices
                J   (6,:,:) double
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
            arguments
                system      (1,1)   elara.SystemNum

                % Array of relative configurations at time step k
                g_rel_k     (4,4,:) double

                % Array of relative configurations at time step k+1
                g_rel_k1    (4,4,:) double

                % Time step
                h           (1,1)   double
            end
            hInv = 1/h; % Precompute for performance (?)

            % Array of absolute frame velocities
            eta_k = zeros(6, system.nFrames);

            % Recursive computation for all frames
            for iFrm = 1:system.nFrames
                iParent = system.frames.parent(iFrm);
                if iParent > 0
                    eta_k(:,iFrm) = elara.SE3.cayInv( ...
                        elara.SE3.invertMatrix(g_rel_k(:,:,iFrm)) ...
                        * elara.SE3.cay(h*eta_k(:,iParent)) ...
                        * g_rel_k1(:,:,iFrm) ...
                        ) *hInv;
                else
                    eta_k(:,iFrm) = elara.SE3.cayInv( ...
                        elara.SE3.invertMatrix(g_rel_k(:,:,iFrm)) * g_rel_k1(:,:,iFrm) ) *hInv;
                end
            end
        end
    end
end
