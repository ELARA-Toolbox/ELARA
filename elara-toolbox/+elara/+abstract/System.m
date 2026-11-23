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

        % Assignment of link numbers to frame numbers
        % First row is the index of the first frame corresponding to the
        % link, second row is the index of the last frame
        % * For rigid links: Both are equal, link has only one frame
        % * For flexible links: First and last index of the beam node frames
        linkFrameIndices    (2,:) double

        %% Tendon actuation data
        % For continuously or discretely routed tendons

        % Tendon-actuation formulation for each link:
        % 0 = no tendon actuation, 1 = continuous, 2 = discrete.
        tendonActuationType (1,:) double {mustBeMember(tendonActuationType, [0,1,2])}

        % Number of tendon-routing disks on each link.
        % Size (1, nLinks).
        nTendonDisksInLink (1,:) double

        % Link-local arc length at each global disk index.
        % Size (1, nDisksTotal), where nDisksTotal counts all disks in the System.
        sTendonDisk        (1,:) double

        % Rows index tendons within a link. Columns index disks globally.
        % An entry is true if the tendon spans the interval from the
        % indexed disk to the next disk on that link.
        % Each link's last disk has an all-false column.
        % Size (nTendonsMax, nDisksTotal), where nTendonsMax is the
        % largest tendon count across links.
        tendonIsActive (:,:) logical

        %% TCP data

        % Frame, to which the TCP is fixed (0 = no TCP defined)
        indexTCPFrame (1,1) uint16 = 0;

        % Transformation from the frame to the TCP
        g_B_TCP     (4,4)  double {elara.internal.validation.mustBeSE3Matrix} = eye(4);
    end

    properties (Hidden)
        % Global disk index range for each link.
        % Size (2, nLinks).
        iTendonDisksInLink (2,:) double

        % Routing section count per link. For continuous routing, this
        % equals the link's nSegments.
        % Size (1, nLinks).
        nTendonSectionsInLink     (1,:) double

        % Reference backbone arc length per global routing section.
        % Size (1, nTendonSections), with nTendonSections the total count.
        lTendonSection           (1,:) double

        % Global beam-frame index per global routing section.
        % Size (1, nTendonSections).
        tendonSectionFrameIndex  (1,:) double

        % Global index range of the sections between two adjacent spacer discs
        % The column number is the global index of the first disk.
        % Both entries are zero for the last disk of each link.
        % Size (2, nDisksTotal).
        iTendonSectionsBetweenDisks   (2,:) double

        % Number of sections between two adjacent disks, stored at the global index
        % of its starting disk.
        % The last disk of each link has a count of zero.
        % Size (1, nDisksTotal).
        nTendonSectionsBetweenDisks   (1,:) double
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
        computeInputMatrixFast
        computeMassMatrixFast
        computeMassMatrix
        computeDiscreteAbsoluteVelocitiesFast
    end

    methods (Abstract, Hidden)
        computeTendonInputMatrixElementContinuous
        computeTendonInputMatrixElementDiscrete
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
