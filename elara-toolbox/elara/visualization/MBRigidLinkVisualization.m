classdef MBRigidLinkVisualization < MBLinkVisualization
    %% Class to visualize a rigid link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties (SetObservable)
        % Draw the link's reference/body frame
        ShowLinkFrame   (1,1) matlab.lang.OnOffSwitchState = true;
    end
    properties (SetAccess=protected)
        % Transform object that specifies the link's configuration
        transf  (1,1) matlab.graphics.primitive.Transform

        % Coordinate system objects for the reference frame(s)
        cSysRef     (1,1) coordSysSE3

        % Transform object for the bounding box (relative to link COM)
        transfBB    (1,1) matlab.graphics.primitive.Transform

        % Object of the bounding box patch
        patchBB     (1,1) matlab.graphics.primitive.Patch
    end

    %% Main Methods
    methods
        function obj = MBRigidLinkVisualization(linkDef, g, opts)
            % Construct an instance of this class
            arguments
                linkDef (1,1) MBLinkDefinition = MBLinkDefinition;
                g       (4,4) double = eye(4);

                % Additional drawing options
                opts.ShowJoint      (1,1) matlab.lang.OnOffSwitchState = true;
                opts.ShowTCPFrame   (1,1) matlab.lang.OnOffSwitchState = true;                
                opts.Name           (1,1) string = "";
                opts.ShowLinkFrame  (1,1) matlab.lang.OnOffSwitchState = true;
                opts.Color          (3,1) double = lines(1);
            end

            obj.linkDef       = linkDef;
            obj.ShowJoint     = opts.ShowJoint;
            obj.ShowTCPFrame  = opts.ShowTCPFrame;            
            obj.Name          = opts.Name;
            obj.ShowLinkFrame = opts.ShowLinkFrame;
            obj.Color         = opts.Color;

            obj = drawLink(obj);
            obj.transf.Matrix = g;

            %%% Register listeners to process property changes
            % Superclass properties
            addlistener(obj,'Color','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'Name','PostSet', @(src,evt)obj.onNameChanged);
            addlistener(obj,'ShowJoint','PostSet', @(src,evt)obj.onShowJointChanged);
            addlistener(obj,'ShowTCPFrame','PostSet', @(src,evt)obj.onShowTCPFrameChanged);
            % Class properties
            addlistener(obj,'ShowLinkFrame','PostSet', @(src,evt)obj.onShowLinkFrameChanged);
        end

        function obj = updateConfiguration(obj, g)
            arguments
                obj
                g   (4,4) double
            end
            obj.transf.Matrix = g;
        end

        function obj = drawLink(obj)
            %% Initialize drawing for a rigid link
            % Note: The link is initialized for its configuration being
            % g = eye(4); the current configuration is set afterwards with
            % the transform property
            obj.transf = hgtransform();

            %% Draw Joint and TCP frame
            obj = drawJoint(obj, obj.transf);
            obj = obj.drawTCPFrame(obj.transf);

            %% Draw body (CoM) frame
            obj.cSysRef = coordSysSE3( eye(4), ...
                'scale', 0.1, ...
                'name', sprintf('$C_{%s}$', obj.Name), ...
                'AxisColors', repmat(obj.Color.', [3,1]), ...
                "parent", obj.transf, ...
                "Visible", obj.ShowLinkFrame...
                );

            %% Draw bounding box

            % Initialize transform specifying the bounding box frame
            obj.transfBB = hgtransform(obj.transf);
            obj.transfBB.Matrix = obj.linkDef.g_bbox;

            % Vertices of the 8 corners (XYZ coordinates for each corner)
            bb = obj.linkDef.bBoxSize;
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

            obj.patchBB =  patch("Faces", F, "Vertices", vert, ...
                "FaceColor", obj.Color, "EdgeColor", obj.Color, ...
                "FaceAlpha", 0.2,...
                "Parent", obj.transfBB ...
                );
        end
    end
    
    %% Update methods for property changes
    methods(Access = private)
        function obj = onColorChanged(obj)
            obj.cSysRef.AxisColors   = repmat(obj.Color.', [3,1]);
            obj.cSysJ.AxisColors     = repmat(obj.Color.', [3,1]);
            obj.cSysTCP.AxisColors   = repmat(obj.Color.', [3,1]);
            obj.jointPatch.FaceColor = obj.Color;
            obj.jointPatch.EdgeColor = obj.Color;
            obj.patchBB.FaceColor    = obj.Color;
            obj.patchBB.EdgeColor    = obj.Color;
        end
        function obj = onNameChanged(obj)
        end
        function obj = onShowJointChanged(obj)
        end
        function obj = onShowTCPFrameChanged(obj)
            obj.cSysTCP.Visible = obj.linkDef.hasTCP && obj.ShowTCPFrame;
        end
        function obj = onShowLinkFrameChanged(obj)
        end
    end
end