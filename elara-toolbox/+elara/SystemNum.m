classdef SystemNum < elara.internal.System
    %% elara.internal.System class for numeric variables
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
        function q = setJointAngles(MBSys,theta)
            %% Return coordinate vector with specified joint angles
            % The rest of q is zero.
            arguments
                MBSys  (1,1) elara.SystemNum

                % Vector of joint angles with dimension (nJoints,theta)
                theta      (:,1) double
            end
            assert(size(theta,1) == MBSys.nJoints, "Joint angle vector has incorrect length.");

            % Get indices of frames with screw joints
            thetaIndices = MBSys.frames.qIndices(1,MBSys.frames.jointType == 1);

            % Set coordinates
            q = zeros(MBSys.nDoF,1);
            q(thetaIndices) = theta;
        end
        function q = setLinkDeformations(MBSys, xi, iLink)
            %% Return coordinate vector with specified beam deformations
            arguments
                MBSys  (1,1) elara.SystemNum

                % Array of discrete deformations with size (6,nSeg) for
                % the current link
                xi      (:,:) double

                % Index of the current link
                iLink   (1,1) double
            end

            % Get frames belonging to the link
            if iLink == 1 && MBSys.isCantilever
                % Cantilever link: All frames correspond to beam segments
                linkFrames = MBSys.linkFrameIndices(1, iLink):MBSys.linkFrameIndices(2, iLink);
            else
                % Regular link: First frame has screw joint and does not
                % correspond to a beam segment
                linkFrames = (MBSys.linkFrameIndices(1, iLink)+1):MBSys.linkFrameIndices(2, iLink);
            end

            nSeg = numel(linkFrames);
            assert(size(xi,2) == nSeg, "Wrong dimensions for xi.")

            % Get indices in q belonging to the flexible link
            % Note: We assume all coordinates of the link are stored
            %       consecutively in the coordinate vector q
            qIndices = MBSys.frames.qIndices(1,linkFrames(1)):MBSys.frames.qIndices(2,linkFrames(end));

            % Store coordinates in q
            % Note: We assume all segments have the same Ba matrix
            Ba = MBSys.frames.Ba(linkFrames(1));
            psi = Ba.' * xi;
            q = zeros(MBSys.nDoF,1);
            q(qIndices) = psi(:);
        end

        function theta = getJointAngles(MBSys, q)
            %% Return vector of joint angles for given coordinate vector
            arguments
                MBSys  (1,1) elara.SystemNum

                % Vector of generalized coordinates from which the joint
                % angles should be returned
                q      (:,1) double
            end
            % Get indices of frames with screw joints
            thetaIndices = MBSys.frames.qIndices(1,MBSys.frames.jointType == 1);

            % Get angles
            theta = q(thetaIndices);
        end

        function xi = getLinkDeformations(MBSys, q, iLink)
            %% Return array of discrete deformations for given link and corrdinate vector
            arguments
                MBSys  (1,1) elara.SystemNum

                % Vector of generalized coordinates from which the
                % discrete deformations shall be computed
                q      (:,1) double

                % Link for which to return the discrete deformations
                iLink  (1,1) double
            end

            % Get frames belonging to the link
            if iLink == 1 && MBSys.isCantilever
                % Cantilever link: All frames correspond to beam segments
                linkFrames = MBSys.linkFrameIndices(1, iLink):MBSys.linkFrameIndices(2, iLink);
            else
                % Regular link: First frame has screw joint and does not
                % correspond to a beam segment
                linkFrames = (MBSys.linkFrameIndices(1, iLink)+1):MBSys.linkFrameIndices(2, iLink);
            end

            nSeg = numel(linkFrames);

            % Get indices in q belonging to the flexible link
            % Note: We assume all coordinates of the link are stored
            %       consecutively in the coordinate vector q
            qIndices = MBSys.frames.qIndices(1,linkFrames(1)):MBSys.frames.qIndices(2,linkFrames(end));

            % Get coordinates and store them in array of size
            % (nAllwd,nSeg)
            % Note: We assume all segments have same nr. of dof/allowed
            %       modes
            nAllwd = MBSys.frames.nDof(linkFrames(1));
            psi = reshape(q(qIndices), nAllwd, nSeg);

            % Compute complete array of deformations
            % Note: We assume all segments have the same Ba matrix
            Ba = MBSys.frames.Ba(linkFrames(1));
            xi = Ba * psi + MBSys.frames.xiC(:,linkFrames);
        end

        function g_rel = computeJointTransformations(MBSys,q)
            %% Compute the relative transformations of all joints (rigid and flexible)
            % for given relative coordinates
            arguments (Input)
                MBSys     (1,1) elara.SystemNum

                % System coordinates  (nDoF, 1)
                q       (:,1) double
            end
            arguments (Output)
                % SE3 Matrices with relative configurations between body frames
                g_rel   (4,4,:) double
            end
            g_rel = zeros(4,4,MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames
                qi = q(MBSys.frames.qIndices(1,iFrm):MBSys.frames.qIndices(2,iFrm));
                switch MBSys.frames.jointType(iFrm)
                    case 1
                        %%% Screw joint
                        g_rel(:,:,iFrm) = MBSys.frames.g_ref(:,:,iFrm) * expSE3Screw(MBSys.frames.X(:,iFrm), qi);
                    case 2
                        %%% Flexible joint
                        xi = MBSys.frames.Ba(iFrm) * qi + MBSys.frames.xiC(:,iFrm);
                        g_rel(:,:,iFrm) = caySE3(xi*MBSys.frames.l(iFrm));
                    otherwise
                        error("Invalid joint type specified.");
                end
            end
        end

        function g = computeFwdKinFast(MBSys, g_rel)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            % with *given* relative joint transformations g_ij
            arguments (Input)
                MBSys     (1,1) elara.SystemNum

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
            g = zeros(4,4,MBSys.nFrames);

            % First frame
            g(:,:,1) = MBSys.g0 * g_rel(:,:,1);

            % Other frames
            for iFrm = 2:MBSys.nFrames
                g(:,:,iFrm) = g(:,:,MBSys.frames.parent(iFrm)) * g_rel(:,:,iFrm);
            end
        end

        function [g, g_rel] = computeFwdKin(MBSys, q)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            arguments (Input)
                MBSys     (1,1) elara.SystemNum

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
            g_rel = MBSys.computeJointTransformations(q);

            % Compute kinematics
            g = computeFwdKinFast(MBSys, g_rel);
        end

        function J = computeGeomJacobianFast(MBSys, q, g_rel)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            % with *given* relative joint transformations g_ij
            arguments (Input)
                MBSys     (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q       (:,1) double

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel   (4,4,:) double
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x (nAllwd_1*nSeg1 + ... + nAllwdB*nSegB + nLinks) x nFrames
                % where B is the nr. of flexible beams in the system
                J       (6,:,:) double
            end

            % Array holding all Jacobians
            J = zeros(6, MBSys.nDoF, MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames
                for ii = 1:iFrm
                    % Column indices of the current block
                    qIndices = MBSys.frames.getQIndices(ii);

                    % Compute block columns for current frame
                    if ii == iFrm
                        switch MBSys.frames.jointType(iFrm)
                            case 1
                                J(:,qIndices,iFrm) = MBSys.frames.X(:,iFrm);
                            case 2
                                xi = MBSys.frames.Ba(iFrm) * q(qIndices) + MBSys.frames.xiC(:,iFrm);
                                J(:,qIndices,iFrm) = ...
                                    MBSys.frames.l(iFrm) * cayRTDSE3( -xi * MBSys.frames.l(iFrm) ) ...
                                    * MBSys.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ii < iFrm && ismember(ii, MBSys.frames.ancestors(:,iFrm))
                            J(:,qIndices,iFrm) = lAdSE3Inv( g_rel(:,:,iFrm) ) ...
                                * J(:,qIndices,MBSys.frames.parent(iFrm));
                        end
                    end
                end
            end
        end

        function [J, g_rel] = computeGeomJacobian(MBSys, q)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            arguments (Input)
                MBSys     (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q       (:,1) double
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x (nAllwd_1*nSeg1 + ... + nAllwdB*nSegB + nLinks) x nFrames
                % where B is the nr. of flexible beams in the system
                J       (6,:,:) double

                % Relative configurations between body frames
                g_rel    (4,4,:) double
            end
            % Compute relative joint transformations
            g_rel = MBSys.computeJointTransformations(q);

            % Compute Jacobians
            J = MBSys.computeGeomJacobianFast(q, g_rel);
        end

        function J_dot = computeGeomJacobianTimeDerivativeFast(MBSys, q, q_dot, eta, g_rel)
            %% Compute the Time Derivative of the Geometric Jacobian Matrix
            % for the full multibody system
            % "Fast" function - with given relative transformations g_ij
            arguments (Input)
                MBSys         (1,1) elara.SystemNum

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
                % Derivative of Jacobian matrix
                J_dot       (6,:,:) double
            end

            % Array holding all Jacobians
            J_dot = zeros(6, MBSys.nDoF, MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames
                for ii = 1:iFrm
                    % Column indices of the current block
                    qIndices = MBSys.frames.getQIndices(ii);

                    % Compute block columns for current frame
                    if ii == iFrm
                        switch MBSys.frames.jointType(iFrm)
                            case 1
                                J_dot(:,qIndices,iFrm) = ...
                                    sadSE3(eta(:,ii)) * MBSys.frames.X(:,iFrm);
                            case 2
                                xi     = MBSys.frames.Ba(iFrm) * q(qIndices) + MBSys.frames.xiC(:,iFrm);
                                xi_dot = MBSys.frames.Ba(iFrm) * q_dot(qIndices);
                                l = MBSys.frames.l(iFrm);
                                J_dot(:,qIndices,iFrm) = l * ( ...
                                    + sadSE3(eta(:,ii)) * cayRTDSE3(-xi*l) ...
                                    + cayRTDSE3dt(-xi*l, -xi_dot*l ) ...
                                    ) * MBSys.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ismember(ii, MBSys.frames.ancestors(:,iFrm))
                            J_dot(:,qIndices,iFrm) = lAdSE3Inv( g_rel(:,:,iFrm) ) ...
                                * J_dot(:,qIndices,MBSys.frames.parent(iFrm));
                        end
                    end
                end
            end
        end

        function [J_dot, g_rel] = computeGeomJacobianTimeDerivative(MBSys, q, q_dot, eta)
            %% Compute the Time Derivative of the Geometric Jacobian Matrix
            % for the full multibody system
            arguments
                MBSys         (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1) double

                % Absolute frame velocities
                eta         (6,:) double
            end

            % Compute relative joint transformations
            g_rel = MBSys.computeJointTransformations(q);

            % Compute actual Jacobian derivative
            J_dot = computeGeomJacobianTimeDerivativeFast(MBSys, q, q_dot, eta, g_rel);
        end

        function B = computeInputMatrixFast(MBSys, g_rel)
            %% Compute the system input matrix
            % "Fast" function -- with given relative deformations
            arguments
                MBSys         (1,1) elara.SystemNum

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel        (4,4,:) double
            end

            B = zeros(MBSys.nDoF, MBSys.nInputs);
            for iFrm = 1:MBSys.nFrames
                if MBSys.frames.uIndices(1,iFrm)
                    uIndices = MBSys.frames.getUIndices(iFrm);
                    qIndices = MBSys.frames.getQIndices(iFrm);

                    switch MBSys.frames.jointType(iFrm)
                        case 1
                            % Rigid joint (scalar input)
                            B(qIndices, uIndices) = 1;
                        case 2
                            % Flexible joint (multiple cable inputs)
                            l = MBSys.frames.l(iFrm);

                            for iC = 1:length(uIndices)
                                % Cable configurations at adjacent nodes
                                g_cm_i1 = MBSys.frames.g_cm(:,:,1,iFrm,iC);
                                g_cm_i2 = MBSys.frames.g_cm(:,:,2,iFrm,iC);

                                % Discrete deformation gradient cable routing
                                % Tangent vector is in elements 4:6
                                xi_c = cayInvSE3( g_cm_i1 \ g_rel(:,:,iFrm) * g_cm_i2 ) / l;

                                % Compute matrix entry
                                b_i = MBSys.frames.Ba(iFrm).' * [
                                    1/2 * ( skew( g_cm_i1(1:3,4) + g_cm_i2(1:3,4) ) ) * xi_c(4:6);
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
        function B = computeInputMatrix(MBSys, q)
            %% Compute the system input matrix
            arguments
                MBSys         (1,1) elara.SystemNum

                % System coordinates (nDoF, 1)
                q           (:,1) double
            end
            % Compute relative joint transformations
            g_rel = MBSys.computeJointTransformations(q);

            % Compute input matrix
            B = MBSys.computeInputMatrixFast(g_rel);
        end

        function M = computeMassMatrixFast(MBSys, J)
            %% Compute the system mass matrix
            arguments (Input)
                MBSys         (1,1) elara.SystemNum

                % Array of geometric Jacobians
                J       (6,:,:) double
            end
            arguments (Output)
                % System mass matrix
                M   (:,:) double
            end
            M = zeros(MBSys.nDoF);
            for iFrm = 1:MBSys.nFrames
                M = M + J(:,:,iFrm).' * MBSys.frames.MGen(:,:,iFrm) * J(:,:,iFrm);
            end
        end

        function [M, J] = computeMassMatrix(MBSys, q)
            %% Compute the system mass matrix
            arguments (Input)
                MBSys         (1,1) elara.SystemNum

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
            J = MBSys.computeGeomJacobian(q);

            % Compute mass matrix
            M = computeMassMatrixFast(MBSys, J);
        end

        function eta_k = computeDiscreteAbsoluteVelocities(MBSys, g_rel_k, g_rel_k1, h)
            %% Compute discrete absolute velocities in the interval (k,k+1)
            % i.e., compute absolute body-fixed velocities eta in se3
            % (vector form) from given relative transformations at time
            % instances k and k+1
            arguments
                MBSys         (1,1)   elara.SystemNum

                % Array of relative configurations at time step k
                g_rel_k     (4,4,:) double

                % Array of relative configurations at time step k+1
                g_rel_k1    (4,4,:) double

                % Time step
                h           (1,1)   double
            end
            hInv = 1/h; % Precompute for performance (?)

            % Array of absolute frame velocities
            eta_k = zeros(6, MBSys.nFrames);

            % Recursive computation for all frames
            for iFrm = 1:MBSys.nFrames
                if iFrm > 1
                    eta_k(:,iFrm) = cayInvSE3( ...
                        invSE3Matrix(g_rel_k(:,:,iFrm)) * caySE3(h*eta_k(:,iFrm-1)) * g_rel_k1(:,:,iFrm) ...
                        ) *hInv;
                else
                    eta_k(:,iFrm) = cayInvSE3( ...
                        invSE3Matrix(g_rel_k(:,:,iFrm)) * g_rel_k1(:,:,iFrm) ) *hInv;
                end
            end
        end
    end
end