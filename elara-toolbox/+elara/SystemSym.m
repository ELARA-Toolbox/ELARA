classdef SystemSym < elara.internal.System
    %% elara.SystemSym class for symbolic variables
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
        function q = setJointAngles(MBSys,theta)
            %% Return coordinate vector with specified joint angles
            % The rest of q is zero.
            arguments
                MBSys  (1,1) elara.SystemSym

                % Vector of joint angles with dimension (nJoints,1)
                theta      (:,1)
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
                MBSys  (1,1) elara.SystemSym

                % Array of discrete deformations with size (6,nSeg) for
                % the current link
                xi      (:,:)

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
                MBSys  (1,1) elara.SystemSym

                % Vector of generalized coordinates from which the joint
                % angles should be returned
                q      (:,1)
            end
            % Get indices of frames with screw joints
            thetaIndices = MBSys.frames.qIndices(1,MBSys.frames.jointType == 1);

            % Get angles
            theta = q(thetaIndices);
        end

        function xi = getLinkDeformations(MBSys, q, iLink)
            %% Return array of discrete deformations for given link and coordinate vector
            arguments
                MBSys  (1,1) elara.SystemSym

                % Vector of generalized coordinates from which the
                % discrete deformations shall be computed
                q      (:,1)

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
            qIndices = double( ...
                MBSys.frames.qIndices(1,linkFrames(1)):MBSys.frames.qIndices(2,linkFrames(end)) ...
                );

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
                MBSys     (1,1) elara.SystemSym

                % System coordinates  (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % SE3 Matrices with relative configurations between body frames
                g_rel   (:,1) SE3
            end
            f = getSE3Functions(q);
            g_rel = createArray(MBSys.nFrames,1, "SE3");
            for iFrm = 1:MBSys.nFrames
                indices = double(MBSys.frames.qIndices(1,iFrm):MBSys.frames.qIndices(2,iFrm)); % Casadi fix: Explicitly convert to double; unit16 integers do not work as indices
                qi = q(indices);
                switch MBSys.frames.jointType(iFrm)
                    case 1
                        %%% Screw joint
                        g_screw = SE3;
                        g_ref = SE3(MBSys.frames.g_ref(1:3,1:3,iFrm),MBSys.frames.g_ref(1:3,4,iFrm));
                        [g_screw.R, g_screw.x] = f.expSE3Screw(MBSys.frames.X(1:3,iFrm), MBSys.frames.X(4:6,iFrm), qi);
                        g_rel(iFrm) = g_ref * g_screw;
                    case 2
                        %%% Flexible joint
                        Ba = MBSys.frames.Ba(iFrm);
                        om = (Ba(1:3,:) * qi + MBSys.frames.xiC(1:3,iFrm));
                        v  = (Ba(4:6,:) * qi + MBSys.frames.xiC(4:6,iFrm));
                        [g_rel(iFrm).R, g_rel(iFrm).x] = f.caySE3(om*MBSys.frames.l(iFrm), v*MBSys.frames.l(iFrm));
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
                MBSys     (1,1) elara.SystemSym

                % Array of relative configurations between body frames
                % dimensions (nFrames, 1)
                g_rel   (:,1) SE3
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (:,1) SE3
            end

            %% Compute kinematics
            % Kinematics without Joint Frames, Section 2.3 (CoM Frames = Body Frames only)
            g = createArray(MBSys.nFrames,1, "SE3");

            % First frame
            g0 = SE3(MBSys.g0(1:3,1:3), MBSys.g0(1:3,4));
            g(1) = g0 * g_rel(1);

            % Other frames
            for iFrm = 2:MBSys.nFrames
                g(iFrm) = g(MBSys.frames.parent(iFrm)) * g_rel(iFrm);
            end
        end

        function [g, g_rel] = computeFwdKin(MBSys, q)
            %% Compute Kinematics for Full Multibody System
            % i.e., the configuration of all body frames (= CoM frames / node
            % frames)
            arguments (Input)
                MBSys     (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % Absolute configurations of all body frames
                g       (:,1) SE3

                % Relative configurations between body frames (joint
                % transformations)
                g_rel   (:,1) SE3
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
                MBSys     (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel   (:,1) SE3
            end
            arguments (Output)
                % Cell array with dimensions nFrames x nFrames
                % Each row contains the individual blocks of the full
                % Jacobian matrix corresponding to the frame
                J       (:,:) cell
            end
            f = getSE3Functions(q);

            % Array holding all Jacobians
            J = cell(MBSys.nFrames, MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames
                for iBlock = 1:iFrm
                    % Column indices of the current block
                    qIndices = double(MBSys.frames.getQIndices(iBlock));

                    % Compute block columns for current frame
                    if iBlock == iFrm
                        switch MBSys.frames.jointType(iFrm)
                            case 1
                                J{iFrm, iBlock} = MBSys.frames.X(:,iFrm);
                            case 2
                                Ba = MBSys.frames.Ba(iFrm);
                                om = (Ba(1:3,:) * q(qIndices) + MBSys.frames.xiC(1:3,iFrm))*MBSys.frames.l(iFrm);
                                v  = (Ba(4:6,:) * q(qIndices) + MBSys.frames.xiC(4:6,iFrm))*MBSys.frames.l(iFrm);
                                J{iFrm, iBlock} = ...
                                    MBSys.frames.l(iFrm) * f.cayRTDSE3( -om, -v ) ...
                                    * MBSys.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if iBlock < iFrm && ismember(iBlock, MBSys.frames.ancestors(:,iFrm))
                            J{iFrm, iBlock} = f.lAdSE3Inv( g_rel(iFrm).R, g_rel(iFrm).x ) ...
                                * J{MBSys.frames.parent(iFrm),iBlock};
                        end
                    end
                end
            end
        end

        function [J, g_rel] = computeGeomJacobian(MBSys, q)
            %% Compute Geometric Jacobian Matrix for Full Multibody System
            arguments (Input)
                MBSys     (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q       (:,1)
            end
            arguments (Output)
                % Jacobian matrices with dimensions
                % 6 x (nAllwd_1*nSeg1 + ... + nAllwdB*nSegB + nLinks) x nFrames
                % where B is the nr. of flexible beams in the system
                J       (:,:) cell

                % Relative configurations between body frames
                g_rel   (:,1) SE3
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
                MBSys         (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)

                % Absolute frame velocities
                eta         (:,1) cell

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel       (:,1) SE3
            end
            arguments (Output)
                % Derivative of Jacobian matrix
                J_dot        (:,:) cell
            end

            %%%% TODO:
            %%%% Decide if we keep eta as six-dimensional or use separate
            %%%% omega, v

            f = getSE3Functions(q);
            fcayRTDSE3dt = getCayRTDSE3dtFunction(q);

            % Array holding all Jacobians
            J_dot = cell(MBSys.nFrames, MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames
                for iBlock = 1:iFrm
                    % Column indices of the current block
                    qIndices = double(MBSys.frames.getQIndices(iBlock));

                    % Compute block columns for current frame
                    if iBlock == iFrm
                        switch MBSys.frames.jointType(iFrm)
                            case 1
                                J_dot{iFrm, iBlock} = ...
                                    f.sadSE3(eta{iBlock}(1:3),eta{iBlock}(4:6)) * MBSys.frames.X(:,iFrm);
                            case 2
                                l = MBSys.frames.l(iFrm);
                                Ba = MBSys.frames.Ba(iFrm);
                                om = (Ba(1:3,:) * q(qIndices) + MBSys.frames.xiC(1:3,iFrm))*l;
                                v  = (Ba(4:6,:) * q(qIndices) + MBSys.frames.xiC(4:6,iFrm))*l;
                                om_dot = Ba(1:3,:) * q_dot(qIndices)*l;
                                v_dot  = Ba(4:6,:) * q_dot(qIndices)*l;

                                J_dot{iFrm, iBlock} = l * ( ...
                                    f.sadSE3(eta{iBlock}(1:3), eta{iBlock}(4:6)) * f.cayRTDSE3(-om, -v) ...
                                    + fcayRTDSE3dt(-om, -v, -om_dot, -v_dot ) ...
                                    ) * MBSys.frames.Ba(iFrm);
                            otherwise
                                % error
                        end
                    else
                        if ismember(iBlock, MBSys.frames.ancestors(:,iFrm))
                            J_dot{iFrm, iBlock} = f.lAdSE3Inv( g_rel(iFrm).R, g_rel(iFrm).x ) ...
                                * J_dot{MBSys.frames.parent(iFrm),iBlock};
                        end
                    end
                end
            end
        end

        function [J_dot, g_rel] = computeGeomJacobianTimeDerivative(MBSys, q, q_dot, eta)
            %% Compute the Time Derivative of the Geometric Jacobian Matrix
            % for the full multibody system
            arguments
                MBSys         (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)

                % System coordinate velocities (nDoF, 1)
                q_dot       (:,1)

                % Absolute frame velocities
                eta         (:,1) cell
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
                MBSys         (1,1) elara.SystemSym

                % Array of relative configurations between body frames
                % dimensions (4,4,nFrames)
                g_rel       (:,1) SE3
            end
            f = getSE3Functions(g_rel(1).R);

            B = cell(MBSys.nFrames, MBSys.nInputs);

            for iFrm = 1:MBSys.nFrames
                if MBSys.frames.uIndices(1,iFrm)
                    uIndices = double(MBSys.frames.getUIndices(iFrm));
                    %qIndices = double(MBSys.frames.getQIndices(iFrm));

                    switch MBSys.frames.jointType(iFrm)
                        case 1
                            % Rigid joint (scalar input)
                            B{iFrm, uIndices} = 1;
                        case 2
                            % Flexible joint (multiple cable inputs)
                            l = MBSys.frames.l(iFrm);

                            for iC = 1:length(uIndices)
                                % Cable configurations at adjacent nodes
                                g_cm_i1 = MBSys.frames.g_cm(1,iFrm,iC);
                                g_cm_i2 = MBSys.frames.g_cm(2,iFrm,iC);

                                % Discrete deformation gradient cable routing
                                % Tangent vector is in elements 4:6
                                g_rel_c = g_cm_i1 \ g_rel(iFrm) * g_cm_i2;
                                [~, v_c] = f.cayInvSE3( g_rel_c.R, g_rel_c.x );
                                v_c = v_c / l;

                                % Compute matrix entry
                                B{iFrm, uIndices(iC)} = ...
                                    -l / norm(v_c) * MBSys.frames.Ba(iFrm).' * [
                                    1/2 * ( skewSO3( g_cm_i1.x + g_cm_i2.x ) ) * v_c;
                                    v_c
                                    ];
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
                MBSys         (1,1) elara.SystemSym

                % System coordinates (nDoF, 1)
                q           (:,1)
            end
            % Compute relative joint transformations
            g_rel = MBSys.computeJointTransformations(q);

            % Compute input matrix
            B = MBSys.computeInputMatrixFast(g_rel);
        end

        function M = computeMassMatrixFast(MBSys, J)
            %% Compute the system mass matrix
            arguments (Input)
                MBSys     (1,1) elara.SystemSym

                % Cell array of geometric Jacobians
                J       (:,:) cell
            end
            arguments (Output)
                % System mass matrix
                M       (:,:) cell
            end
            % Compute blocks of M based on dyadic product J.' * M * J
            M = cell(MBSys.nFrames, MBSys.nFrames);
            for iFrm = 1:MBSys.nFrames % Sum over all the frame contributions
                for iRow = 1:MBSys.nFrames % Rows of M
                    for iCol = 1:MBSys.nFrames % Columns of M
                        if ~isempty(J{iFrm,iRow}) && ~isempty(J{iFrm,iCol})
                            MBlock = J{iFrm,iRow}.' * MBSys.frames.MGen{iFrm} * J{iFrm,iCol};
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

        function [M, J] = computeMassMatrix(MBSys, q)
            %% Compute the system mass matrix
            arguments (Input)
                MBSys     (1,1) elara.SystemSym

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
            J = MBSys.computeGeomJacobian(q);

            % Compute mass matrix
            M = computeMassMatrixFast(MBSys, J);
        end

        function eta_k = computeDiscreteAbsoluteVelocities(MBSys, g_rel_k, g_rel_k1, h)
            %% Compute discrete absolute velocities in the interval (k,k+1)
            % i.e., compute absolute body-fixed velocities eta in se3
            % (vector form) from given relative transformations at time
            % instances k and k+1
            arguments (Input)
                MBSys         (1,1) elara.SystemSym

                % Cell array of relative configurations at time step k
                g_rel_k     (:,1) SE3

                % Cell array of relative configurations at time step k+1
                g_rel_k1    (:,1) SE3

                % Time step
                h           (1,1)
            end
            arguments (Output)
                % Array of absolute frame velocities
                % (separate rotational and translational components)
                eta_k       (:,2) cell
            end
            f = getSE3Functions(g_rel_k(1).R);

            eta_k = cell(MBSys.nFrames, 2);

            % Recursive computation for all frames
            for iFrm = 1:MBSys.nFrames
                if iFrm > 1
                    g_eta = SE3;
                    [g_eta.R, g_eta.x] = f.caySE3(h*eta_k{iFrm-1, 1}, h*eta_k{iFrm-1, 2});
                    g_rel = g_rel_k(iFrm) \ g_eta * g_rel_k1(iFrm);
                else
                    g_rel = g_rel_k(iFrm) \ g_rel_k1(iFrm);
                end
                [om_k, v_k] = f.cayInvSE3(g_rel.R, g_rel.x);
                eta_k{iFrm,1} = om_k / h;
                eta_k{iFrm,2} = v_k / h;
            end
        end
    end
end