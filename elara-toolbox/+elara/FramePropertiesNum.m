classdef FramePropertiesNum < elara.abstract.FrameProperties
    %% FrameProperties Class (numeric representation)
    % Class that holds all frame-related data for all abstract frames
    % in a multibody system, e.g., joint kinematics and inertia properties.
    properties
        %% Inertia properties
        % Generalized inertia tensor
        MGen    (6,6,:) double {elara.internal.validation.mustBeSymmetricPosDefinite};

        % Frame mass (kg)
        m       (:,1) double {mustBeNonnegative}

        % Additional masses attached to a frame that are offset from the COM
        % (mainly to include additional bodies attached to the beam)
        % Note: They are *additionally* included in MGen and m
        % (where the inertia tensor in MGenFrame is expressed w.r.t. the
        % frame origin = Body COM)

        % Transformation to the external body's CoM
        g_a     (4,4,:) double {elara.internal.validation.mustBeSE3Matrix}

        % Position vector to the external body's CoM (corresponding to g_a)
        x_a     (3,:) double

        % Mass of the external body
        m_a     (1,:) double {mustBeNonnegative}

        % For tendon actuation:
        % Relative SE(3) configurations of the cable path frames (w.r.t.
        % backbone) at the two segment nodes (i.e., nodes i-1 and i for
        % flex. joint i) (in beam reference configuration)
        % Dimensions: (4, 4, 2, nFrames, nTendonsMax),
        % where nTendonsMax is the largest number of tendons across all
        % tendon-actuated flexible links
        g_cm        (4,4,2,:,:) double
    end
end
