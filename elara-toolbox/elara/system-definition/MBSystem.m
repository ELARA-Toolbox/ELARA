classdef (Abstract) MBSystem
    %% MBSystem class
    % Specifies a complete multibody system in tree topology consisting
    % of several rigid or flexible links.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %% General Properties

        % Whether the system's first link is a cantilever (= beam with
        % fixed first node) or not (= first link mounted with joint)
        isCantilever        (1,1) logical = false;

        % Absolute configuration of the first joint
        g0                  (4,4) double {mustBeSE3Matrix} = eye(4);


        %% Graph Adjacency Matrices
        AdjMatrixLinkGraph    (:,:) double
        AdjMatrixFrameGraph   (:,:) double


        %% System Topology Data

        % Nr. of links in the system
        nLinks              (1,1) double

        % Nr. of 1dof joints in the system
        nJoints             (1,1) double

        % Nr. of frames in the system (including node frames from flexible
        % beams)
        nFrames             (1,1) double

        % Total nr. of DoFs in the system
        nDoF                (1,1) double

        % Nr. of system inputs (control inputs)
        nInputs             (1,1) double

        % Assignment of link numbers to frame numbers
        % First row is the index of the first frame corresponding to the
        % link, second row is the index of the last frame
        % * For rigid links: Both are equal, link has only one frame
        % * For flexible links: First and last index of the beam node frames
        linkFrameIndices    (2,:) uint16


        %% TCP data

        % Frame, to which the TCP is fixed (0 = no TCP defined)
        indexTCPFrame (1,1) uint16 = 0;

        % Transformation from the frame to the TCP
        g_B_TCP     (4,4)  double {mustBeSE3Matrix} = eye(4);
    end

    properties (Abstract)
        % frameData
        % cSys
        % dSys
        % dSys
    end

    methods (Abstract)
        setJointAngles
        setLinkDeformations
        getJointAngles
        getLinkDeformations
        computeJointTransformations
        computeFwdKinFast
        computeFwdKin
        computeGeomJacobianFast
        computeGeomJacobian
        computeGeomJacobianTimeDerivativeFast
        computeGeomJacobianTimeDerivative
        computeInputMatrixFast
        computeInputMatrix
        computeMassMatrixFast
        computeMassMatrix
        computeDiscreteAbsoluteVelocities
    end
    methods
        function obj = MBSystem(links)
            %% Constructor for MBSystem
            arguments
                links (:,1) MBLinkDefinition = MBLinkDefinitionRigid.empty;
            end
            % Assemble system if links are given
            if ~isempty(links)
                obj = assembleMBSystem(links, obj);
            end
        end
    end
end