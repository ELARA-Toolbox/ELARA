classdef (Abstract) Link < matlab.mixin.Heterogeneous
    % Abstract Class defining a general link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    properties
        %% Joint properties

        % Parent link index (0 = first link / attached to fixed base)
        parentLink      (1,1) uint16 {mustBeNonnegative}

        % If the joint is actuated (with a generalized force/torque)
        jointIsActuated (1,1) logical = true;

        % Joint screw coordinate vector in joint frame
        jointAxis       (6,1) double

        % Transformation from parent link reference frame to current body's
        % reference frame in reference configuration
        g_ref           (4,4) double {mustBeSE3Matrix} = eye(4);

        % Transformation Joint --> Body reference frame
        % * For rigid links:    Joint 1 -> CoM
        % * For flexible links: Joint 1 -> First node
        g_J_B           (4,4) double {mustBeSE3Matrix} = eye(4);

        % Joint dissipation coefficient
        d           (1,1) double {mustBeNonnegative}

        % Joint stiffness coefficient
        c           (1,1) double {mustBeNonnegative}


        %%% Properties of additional bodies attached to the link
        % For flexible beams:
        % Must include the mounting bodies at the start and end of the
        % beam, if existing

        % Array of generalized inertia tensors (w.r.t. the beam center
        % line/body-fixed cross-section frame), dimensions (6,6,nNodes)
        M_a         (6,6,:) double

        % Vector of body masses; dimensions (nNodes,1)
        m_a         (1,:) double {mustBeNonnegative}

        % Array of transformations from the cross-section frame to the COM
        % frame of attached rigid bodies
        g_a         (4,4,:) double {mustBeSE3MatrixArray}

        %% TCP definition
        % To add a TCP to the system (useful for robots)

        % Defines whether the current link contains the robot's TCP
        % (can only be true for one link in the full system)
        % Note: For flexible links, the last link frame is used
        hasTCP      (1,1) logical

        % Transformation from the link's body/COM frame to the TCP
        g_B_TCP     (4,4)  double {mustBeSE3Matrix} = eye(4);
    end
    properties (Abstract, Constant)
        % Whether the link is rigid or flexible
        isRigid (1,1) logical
    end
    methods (Abstract)
        % Method to validate the properties of a link; should throw an
        % error if any property is invalid
        validateProperties()

        % Initialize and return the linkVisualization object for the
        % present link class
        getLinkVisualization()
    end
end