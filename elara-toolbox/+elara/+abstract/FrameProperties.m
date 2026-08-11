classdef (Abstract) FrameProperties
    %% ELARA FrameProperties Class
    % Class that holds all frame-related data for all abstract frames 
    % in a multibody system, e.g., joint kinematics and inertia properties.
    properties
        %% General/kinematic properties

        % Degrees of freedom of the frame joint (1 to 6)
        nDof        (:,1) uint8

        % Joint type
        % 1 = Screw joint (lower-pair joint)
        % 2 = Flexible beam joint
        jointType   (:,1) uint8

        % Coordinate indices of the frame, i.e., elements in q that
        % correspond to the frame
        % First element: first index, Second element: last index.
        qIndices    (2,:) uint16

        % Input indices of the frame, i.e., elements in u that correspond
        % to the frame's inputs.
        % First element: first index, Second element: last index.
        uIndices    (2,:) uint16

        % Index of the link, to which the frame belongs
        linkIndex   (1,:) uint16

        % Index of the parent frame
        parent      (1,:) uint16

        % Vector of ancestor frames, i.e., frames that connect the current
        % frame to the fixed base. Sorted from base to current frame.
        % Note: Padded with zeros, so the overall size is (maxLength,
        % nFrames), where maxLength is the length of the largest consecutive 
        % chain in the system.
        % (First index: Ancestors, Second index: iFrame).
        ancestors   (:,:) uint16

        %% Joint properties for (lower-pair) screw joints
        % Joint screw vector represented in Body (CoM) frame
        X       (6,:) double

        % Transformation from parent reference frame to current frame
        % in reference configuration
        g_ref   (4,4,:) double {elara.internal.validation.mustBeSE3Matrix};

        %% Joint properties for flexible beam joints

        % Selection matrix for allowed strains, padded with zeros to the
        % right (i.e., Ba_padded = [Ba, 0])
        BaPadded    (6,6,:) double

        % Vector of constrained strains, i.e., xi_c = B_c * psi_c
        xiC         (6,:) double

        % Segment length
        l           (:,1) double

        % Whether a tendon is active on a flexible beam segment
        % Dimensions: (nFrames, nTendonsMax)
        tendonIsActive (:,:) logical
    end

    methods
        function BaMat = Ba(obj, iFrm)
            %% Get frame's Ba matrix
            % Helper function to get the Ba matrix with correct dimensions
            % from the stored array padded with zeros
            arguments
                obj
                iFrm % index of the frame to get
            end
            BaMat = obj.BaPadded(:,1:obj.nDof(iFrm),iFrm);
        end

        function indices = getQIndices(obj, iFrm)
            %% Get all indices in the configuration vector for the current frame
            % Helper function to get all indices from the stored first and
            % last index
            indices = obj.qIndices(1,iFrm):obj.qIndices(2,iFrm);
        end
        function indices = getUIndices(obj, iFrm)
            %% Get all indices in the input vector for the current frame
            % Helper function to get all indices from the stored first and
            % last index
            indices = obj.uIndices(1,iFrm):obj.uIndices(2,iFrm);
        end
    end
end
