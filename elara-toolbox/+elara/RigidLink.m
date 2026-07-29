classdef RigidLink < elara.abstract.Link
    % Class defining a Rigid Link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    properties
        %% Rigid Link properties

        % Mass (kg)
        m           (1,1) double {mustBeNonnegative} = 1;

        % Inertia tensor (kgm^2)
        J           (3,3) double {mustBeSymmetricPosDefinite} = eye(3);

        %% Link Visualization Properties (bounding box)

        % Transformation from reference frame to bounding box center
        g_bbox      (4,4) double {mustBeSE3Matrix} = eye(4);

        % Dimensions of the bounding box, measured form g_bbox
        % x+ y+ z+
        % x- y- z-
        bBoxSize    (2,3) double
    end
    properties (Constant)
        isRigid = true;
    end
    methods
        function validateProperties(~)
            % Nothing to do here
        end
        function linkVis = getLinkVisualization(link, varargin)
            %% Initialize and return link visualization object
            linkVis = elara.visualization.RigidLinkVisualization(link, varargin{:});
        end
    end
end