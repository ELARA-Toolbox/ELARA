classdef FlexibleLinkVisualization < elara.abstract.LinkVisualization
    %% Class to visualize a flexible link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties (SetObservable)
        % Draw the beam's tendons (if it has any)?
        showTendons         (1,1) matlab.lang.OnOffSwitchState = true;

        % Show frames of the beam?
        showBeamFrames      (1,1) matlab.lang.OnOffSwitchState = false;
    end
    properties (SetAccess = protected)
        % Objects for the beam joint plots
        % (lines between joints and first / last nodes)
        flexibleLinkJointPlot   (1,1) matlab.graphics.chart.primitive.Line

        % Object for beam visualization
        beamVisualization             (1,1) elara.visualization.ElasticBeam

        % Objects for tendons
        tendonVisualization           (:,1) elara.visualization.ElasticBeam
    end

    %% Main Methods
    methods
        function obj = FlexibleLinkVisualization(link, g, opts)
            % Construct an instance of this class
            arguments
                link (1,1)   elara.FlexibleLink;
                g       (4,4,:) double {mustBeSE3Matrix} = zeros(4,4,0);

                % Additional drawing options
                opts.showJoint      (1,1) matlab.lang.OnOffSwitchState = true;
                opts.showTCPFrame   (1,1) matlab.lang.OnOffSwitchState = true;
                opts.showTendons    (1,1) matlab.lang.OnOffSwitchState = true;
                opts.showBeamFrames (1,1) matlab.lang.OnOffSwitchState = false;
                opts.Color          (3,1) double = lines(1);
                opts.Name           (1,1) string = "";
            end

            obj.link      = link;
            obj.showJoint    = opts.showJoint;
            obj.showTCPFrame = opts.showTCPFrame;
            obj.showTendons  = opts.showTendons;
            obj.showBeamFrames = opts.showBeamFrames;
            obj.Color = opts.Color;
            obj.Name  = opts.Name;

            if ~isempty(g)
                obj = obj.updateConfiguration(g);
            end

            %%% Register listeners to process property changes
            % Superclass properties
            addlistener(obj,'Color','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'Name','PostSet', @(src,evt)obj.onNameChanged);
            addlistener(obj,'showJoint','PostSet', @(src,evt)obj.onShowJointChanged);
            addlistener(obj,'showTCPFrame','PostSet', @(src,evt)obj.onShowTCPFrameChanged);
            % Class properties
            addlistener(obj,'showTendons','PostSet', @(src,evt)obj.onShowTendonsChanged);
            addlistener(obj,'showBeamFrames','PostSet', @(src,evt)obj.onShowBeamFramesChanged);
        end

        function obj = updateConfiguration(obj, g)
            arguments
                obj
                g   (4,4,:) double %{mustBeSE3Matrix}
            end

            if isempty(obj.beamVisualization.crossSectionTransforms)
                obj = obj.drawLink(g);
            end


            % Update beam
            obj.beamVisualization.updateConfiguration(g);

            % Update tendons
            nCables = length(obj.link.tendonActuation.x_td_funs);
            if nCables && obj.showTendons
                lBeam = obj.link.L/obj.link.nSegments;
                sBeamNodes = 0:lBeam:obj.link.L;
                [g_cm, termNodes] = obj.link.tendonActuation.getNodeData( ...
                    sBeamNodes);
                for iC = 1:nCables
                    g_c = zeros(4, 4, termNodes(iC));

                    for iN = 1:termNodes(iC)
                        % Cable points initial configuration
                        g_c(:,:,iN) = g(:,:,iN) * g_cm(:,:,iN,iC);
                    end
                    obj.tendonVisualization(iC).updateConfiguration(g_c);
                end
            end
        end
        function obj = drawLink(obj,g)
            %% Initialize drawing for a flexible link
            arguments
                obj
                g (4,4,:) double {mustBeSE3Matrix}
            end

            % Draw beam
            obj.beamVisualization = elara.visualization.ElasticBeam(g, ...
                "showFrames", obj.showBeamFrames, ...
                "showLabels", obj.showBeamFrames, ...
                "drawCrossSections", true, ...
                "interpolateBeam", true, ...
                "FaceAlpha", 0.3, ...
                "Color",obj.Color, ...
                "Height", obj.link.beamParameters.height, ...
                "Width", obj.link.beamParameters.width ...
                );

            % Draw Joint and TCP frame
            obj = drawJoint(obj, obj.beamVisualization.crossSectionTransforms([1,end]));
            obj = obj.drawTCPFrame(obj.beamVisualization.crossSectionTransforms(end));

            % Links from/to joints
            g_J1_B_inv = inv(obj.link.g_J_B);
            obj.flexibleLinkJointPlot = plot3( ...
                [g_J1_B_inv(1,4), 0], ...
                [g_J1_B_inv(2,4), 0], ...
                [g_J1_B_inv(3,4), 0], ...
                "Color", obj.Color, ...
                "LineWidth",3, ...
                "Parent", obj.beamVisualization.crossSectionTransforms(1) ...
                );

            % Tendons
            nCables = length(obj.link.tendonActuation.x_td_funs);
            if nCables
                colorsTendons = lines(nCables);
                lBeam = obj.link.L/obj.link.nSegments;
                sBeamNodes = 0:lBeam:obj.link.L;
                [g_cm, termNodes] = obj.link.tendonActuation.getNodeData( ...
                    sBeamNodes);

                for iC = 1:nCables
                    g_c = zeros(4, 4, termNodes(iC));

                    for iN = 1:termNodes(iC)
                        % Cable points initial configuration
                        g_c(:,:,iN) = g(:,:,iN) * g_cm(:,:,iN,iC);
                    end

                    obj.tendonVisualization(iC) = elara.visualization.ElasticBeam( g_c, ...
                        "edgeAlpha", 0, ...
                        "FaceAlpha", 0, ...
                        "DrawEdges", false, ...
                        "drawCenterline", true, ...
                        "drawCrossSections", true, ...
                        "showFrames", false, ...
                        "interpolateBeam", true, ...
                        "Color", colorsTendons(iC,:), ...
                        "Height", 0.003, "Width", 0.003, ...
                        "showFrames", false, ...
                        "Visible", obj.showTendons...
                        );
                end
            end
        end
    end
    %% Update methods for property changes
    methods(Access = private)
        function obj = onColorChanged(obj)
            obj.coordSysJ.AxisColors        = repmat(obj.Color.', [3,1]);
            obj.coordSysTCP.AxisColors      = repmat(obj.Color.', [3,1]);            
            obj.flexibleLinkJointPlot.Color = obj.Color;
            obj.jointPatch.FaceColor        = obj.Color;
            obj.jointPatch.EdgeColor        = obj.Color;
            obj.beamVisualization.Color     = obj.Color;
        end
        function obj = onNameChanged(obj)
        end
        function obj = onShowJointChanged(obj)
        end
        function obj = onShowTCPFrameChanged(obj)
            obj.coordSysTCP.Visible = obj.link.hasTCP && obj.showTCPFrame;
        end
        function obj = onShowTendonsChanged(obj)
        end
        function obj = onShowBeamFramesChanged(obj)
        end
    end
end