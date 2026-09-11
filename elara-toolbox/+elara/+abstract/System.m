classdef (Abstract) System
    %% elara.abstract.System class
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
        g0                  (4,4) double {elara.internal.validation.mustBeSE3Matrix} = eye(4);


        %% Graph Adjacency Matrices
        LinkAdjacencyMatrix    (:,:) double
        FrameAdjacencyMatrix   (:,:) double


        %% System Topology Data

        % Number of links in the system
        nLinks              (1,1) double

        % Number of one-DoF joints in the system
        nJoints             (1,1) double

        % Number of frames in the system (including node frames from flexible
        % beams)
        nFrames             (1,1) double

        % Total number of DoFs in the system
        nDoF                (1,1) double

        % Number of system inputs (control inputs)
        nInputs             (1,1) double

        % Tendon-actuation formulation for each link:
        % 1 = continuous, 2 = discrete.
        tendonActuationType (1,:) uint8 {mustBeMember(tendonActuationType, [1,2])}

        % Physical-joint actuation for each link:
        % 0 = not actuated, 1 = actuated.
        jointActuationType (1,:) uint8 {mustBeMember(jointActuationType, [0,1])}
        % Assignment of link numbers to frame numbers
        % First row is the index of the first frame corresponding to the
        % link, second row is the index of the last frame
        % * For rigid links: Both are equal, link has only one frame
        % * For flexible links: First and last index of the beam node frames
        linkFrameIndices    (2,:) uint16

        % Local arc length at the end of each flexible frame section.
        % Entries corresponding to rigid frames are zero.
        sFrames            (1,:) double

        % Number of tendon-routing disks on each link.
        nDisks             (1,:) double

        % Local arc-length positions of tendon-routing disks, padded with zeros.
        sDisks             (:,:) double
        % Length of each integration section, padded with zeros by link.
        sSection                (:,:) double

        % Global frame index associated with each integration section.
        frameIndexSection       (:,:) uint16

        % First and last section index between consecutive disks.
        % Dimensions: (2,nDiskIntervalsMax,nLinks).
        iSectionsBetweenDisks   (2,:,:) uint16

        % Number of integration sections between consecutive disks.
        % Dimensions: (nLinks,nDiskIntervalsMax).
        nSectionsBetweenDisks   (:,:) uint16



        %% TCP data

        % Frame, to which the TCP is fixed (0 = no TCP defined)
        indexTCPFrame (1,1) uint16 = 0;

        % Transformation from the frame to the TCP
        g_B_TCP     (4,4)  double {elara.internal.validation.mustBeSE3Matrix} = eye(4);
    end

    properties (Abstract)
        % frames
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
        computeGeomJacobianAccelerationBiasMatrixFast
        computeGeomJacobianAccelerationBiasMatrix
        computeGeomJacobianTimeDerivativeFast
        computeGeomJacobianTimeDerivative
        computeTendonInputMatrixElementContinuous
        computeTendonInputMatrixElementDiscrete
        computeInputMatrix
        computeMassMatrixFast
        computeMassMatrix
        computeDiscreteAbsoluteVelocities
    end
    methods
        function obj = System(links)
            %% Constructor for elara.abstract.System
            arguments
                links (:,1) elara.abstract.Link = elara.RigidLink.empty;
            end
            % Assemble system if links are given
            if ~isempty(links)
                obj = elara.assembleSystem(links, obj);
            end
        end
    end
end
