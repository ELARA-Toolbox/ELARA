classdef RigidLinkVisualization < elara.internal.LinkVisualization
    %% Class to visualize a rigid link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties (SetObservable)
        % Draw the link's reference/body frame
        showLinkFrame   (1,1) matlab.lang.OnOffSwitchState = true;
    end
    properties (SetAccess=protected)
        % Transform object that specifies the link's configuration
        configurationTransform  (1,1) matlab.graphics.primitive.Transform

        % Coordinate system objects for the reference frame(s)
        coordSysRef     (1,1) CoordSysSE3

        % Transform object for the bounding box (relative to link COM)
        boundingBoxTransform    (1,1) matlab.graphics.primitive.Transform

        % Object of the bounding box patch
        boundingBoxPatch     (1,1) matlab.graphics.primitive.Patch
    end

    %% Main Methods
    methods
        function obj = RigidLinkVisualization(link, g, options)
            % Construct an instance of this class
            arguments
                link    (1,1) elara.RigidLink = elara.RigidLink;
                g       (4,4) double = eye(4);

                % Additional drawing options
                options.showJoint      (1,1) matlab.lang.OnOffSwitchState = true;
                options.showTCPFrame   (1,1) matlab.lang.OnOffSwitchState = true;                
                options.Name           (1,1) string = "";
                options.showLinkFrame  (1,1) matlab.lang.OnOffSwitchState = true;
                options.Color          (3,1) double = lines(1);
            end

            obj.link       = link;
            obj.showJoint     = options.showJoint;
            obj.showTCPFrame  = options.showTCPFrame;            
            obj.Name          = options.Name;
            obj.showLinkFrame = options.showLinkFrame;
            obj.Color         = options.Color;

            obj = drawLink(obj);
            obj.configurationTransform.Matrix = g;

            %%% Register listeners to process property changes
            % Superclass properties
            addlistener(obj,'Color','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'Name','PostSet', @(src,evt)obj.onNameChanged);
            addlistener(obj,'showJoint','PostSet', @(src,evt)obj.onShowJointChanged);
            addlistener(obj,'showTCPFrame','PostSet', @(src,evt)obj.onShowTCPFrameChanged);
            % Class properties
            addlistener(obj,'showLinkFrame','PostSet', @(src,evt)obj.onShowLinkFrameChanged);
        end

        function obj = updateConfiguration(obj, g)
            arguments
                obj
                g   (4,4) double
            end
            obj.configurationTransform.Matrix = g;
        end

        function obj = drawLink(obj)
            %% Initialize drawing for a rigid link
            % Note: The link is initialized for its configuration being
            % g = eye(4); the current configuration is set afterwards with
            % the transform property
            obj.configurationTransform = hgtransform();

            %% Draw Joint and TCP frame
            obj = drawJoint(obj, obj.configurationTransform);
            obj = obj.drawTCPFrame(obj.configurationTransform);

            %% Draw body (CoM) frame
            obj.coordSysRef = CoordSysSE3( eye(4), ...
                'scale', 0.1, ...
                'name', sprintf('$C_{%s}$', obj.Name), ...
                'AxisColors', repmat(obj.Color.', [3,1]), ...
                "parent", obj.configurationTransform, ...
                "Visible", obj.showLinkFrame...
                );

            %% Draw bounding box

            % Initialize transform specifying the bounding box frame
            obj.boundingBoxTransform = hgtransform(obj.configurationTransform);
            obj.boundingBoxTransform.Matrix = obj.link.g_bbox;

            % Vertices of the 8 corners (XYZ coordinates for each corner)
            bb = obj.link.bBoxSize;
            vert = [
                bb(1,1), bb(1,2), bb(1,3)
                bb(1,1), bb(2,2), bb(1,3)
                bb(1,1), bb(2,2), bb(2,3)
                bb(1,1), bb(1,2), bb(2,3)
                bb(2,1), bb(1,2), bb(1,3)
                bb(2,1), bb(2,2), bb(1,3)
                bb(2,1), bb(2,2), bb(2,3)
                bb(2,1), bb(1,2), bb(2,3)
                ];

            % Bounding box faces: Each row connects four corners
            F = [
                1 2 3 4
                5 6 7 8
                1 5 6 2
                2 6 7 3
                1 5 6 2
                3 4 8 7
                ];

            obj.boundingBoxPatch =  patch("Faces", F, "Vertices", vert, ...
                "FaceColor", obj.Color, "EdgeColor", obj.Color, ...
                "FaceAlpha", 0.2,...
                "Parent", obj.boundingBoxTransform ...
                );
        end
    end
    
    %% Update methods for property changes
    methods(Access = private)
        function obj = onColorChanged(obj)
            obj.coordSysRef.AxisColors   = repmat(obj.Color.', [3,1]);
            obj.coordSysJ.AxisColors     = repmat(obj.Color.', [3,1]);
            obj.coordSysTCP.AxisColors   = repmat(obj.Color.', [3,1]);
            obj.jointPatch.FaceColor = obj.Color;
            obj.jointPatch.EdgeColor = obj.Color;
            obj.boundingBoxPatch.FaceColor    = obj.Color;
            obj.boundingBoxPatch.EdgeColor    = obj.Color;
        end
        function obj = onNameChanged(obj)
            % TODO
        end
        function obj = onShowJointChanged(obj)
            % TODO
        end
        function obj = onShowTCPFrameChanged(obj)
            obj.coordSysTCP.Visible = obj.link.hasTCP && obj.showTCPFrame;
        end
        function obj = onShowLinkFrameChanged(obj)
            obj.cSysRef.Visible = obj.ShowLinkFrame;
        end
    end
end