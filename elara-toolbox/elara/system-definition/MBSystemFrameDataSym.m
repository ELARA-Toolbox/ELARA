classdef MBSystemFrameDataSym < MBSystemFrameData
    %% MBSystemFrameData Class (symbolic representation)
    % Class that holds all frame-related data for all abstract frames
    % in a multibody system, e.g., joint kinematics and inertia properties.
    properties
        %% Inertia properties
        % Generalized inertia tensor
        MGen    (:,1) cell

        % Frame mass (kg)
        m       (:,1)

        % Additional masses attached to a frame that are offset from the COM
        % (mainly to include additional bodies attached to the beam)
        % Note: They are *additionally* included in MGen and m
        % (where the inertia tensor in MGenFrame is expressed w.r.t. the
        % frame origin = Body COM)

        % Transformation to the external body's CoM
        g_a     (4,4,:) double {mustBeSE3MatrixArray}

        % Position vector to the external body's CoM (corresponding to g_a)
        x_a     (3,:)

        % Mass of the external body
        m_a     (1,:)

        % For cable actuation:
        % Relative SE3 configurations of the cable path frames (w.r.t.
        % backbone) at the two segment nodes (i.e., nodes i-1 and i for
        % flex. joint i) (in beam reference configuration)
        % Dimensions: (4, 4, 2, nFrames, nCablesMax),
        % where nCablesMax is the largest nr. of cables of all
        % cable-actuated flexible links
        g_cm        (2,:,:) SE3
    end
end