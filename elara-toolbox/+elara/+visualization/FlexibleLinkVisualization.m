classdef FlexibleLinkVisualization < elara.internal.LinkVisualization
    %% Class to visualize a flexible link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties (SetObservable)
        % Draw the beam's tendons (if it has any)?
        ShowTendons         (1,1) matlab.lang.OnOffSwitchState = true;

        % Show frames of the beam?
        ShowBeamFrames      (1,1) matlab.lang.OnOffSwitchState = false;
    end
    properties (SetAccess = protected)
        % Objects for the beam joint plots
        % (lines between joints and first / last nodes)
        flexLinkJointPlot   (1,1) matlab.graphics.chart.primitive.Line

        % Object for beam visualization
        beamVis             (1,1) elara.visualization.elasticBeam

        % Objects for tendons
        tendonVis           (:,1) elara.visualization.elasticBeam
    end

    %% Main Methods
    methods
        function obj = FlexibleLinkVisualization(linkDef, g, opts)
            % Construct an instance of this class
            arguments
                linkDef (1,1)   elara.FlexibleLink;
                g       (4,4,:) double {mustBeSE3MatrixArray} = zeros(4,4,0);

                % Additional drawing options
                opts.ShowJoint      (1,1) matlab.lang.OnOffSwitchState = true;
                opts.ShowTCPFrame   (1,1) matlab.lang.OnOffSwitchState = true;
                opts.ShowTendons    (1,1) matlab.lang.OnOffSwitchState = true;
                opts.ShowBeamFrames (1,1) matlab.lang.OnOffSwitchState = false;
                opts.Color          (3,1) double = lines(1);
                opts.Name           (1,1) string = "";
            end

            obj.linkDef      = linkDef;
            obj.ShowJoint    = opts.ShowJoint;
            obj.ShowTCPFrame = opts.ShowTCPFrame;
            obj.ShowTendons  = opts.ShowTendons;
            obj.ShowBeamFrames = opts.ShowBeamFrames;
            obj.Color = opts.Color;
            obj.Name  = opts.Name;

            if ~isempty(g)
                obj = obj.updateConfiguration(g);
            end

            %%% Register listeners to process property changes
            % Superclass properties
            addlistener(obj,'Color','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'Name','PostSet', @(src,evt)obj.onNameChanged);
            addlistener(obj,'ShowJoint','PostSet', @(src,evt)obj.onShowJointChanged);
            addlistener(obj,'ShowTCPFrame','PostSet', @(src,evt)obj.onShowTCPFrameChanged);
            % Class properties
            addlistener(obj,'ShowTendons','PostSet', @(src,evt)obj.onShowTendonsChanged);
            addlistener(obj,'ShowBeamFrames','PostSet', @(src,evt)obj.onShowBeamFramesChanged);
        end

        function obj = updateConfiguration(obj, g)
            arguments
                obj
                g   (4,4,:) double %{mustBeSE3MatrixArray}
            end

            if isempty(obj.beamVis.transfCS)
                obj = obj.drawLink(g);
            end


            % Update beam
            obj.beamVis.updateConfiguration(g);

            % Update tendons
            nCables = length(obj.linkDef.tendonActuation.x_td_funs);
            if nCables && obj.ShowTendons
                lBeam = obj.linkDef.L/obj.linkDef.nSeg;
                sBeamNodes = 0:lBeam:obj.linkDef.L;
                [g_cm, termNodes] = obj.linkDef.tendonActuation.getNodeData( ...
                    sBeamNodes);
                for iC = 1:nCables
                    g_c = zeros(4, 4, termNodes(iC));

                    for iN = 1:termNodes(iC)
                        % Cable points initial configuration
                        g_c(:,:,iN) = g(:,:,iN) * g_cm(:,:,iN,iC);
                    end
                    obj.tendonVis(iC).updateConfiguration(g_c);
                end
            end
        end
        function obj = drawLink(obj,g)
            %% Initialize drawing for a flexible link
            arguments
                obj
                g (4,4,:) double {mustBeSE3MatrixArray}
            end

            % Draw beam
            obj.beamVis = elara.visualization.elasticBeam(g, ...
                "showFrames", obj.ShowBeamFrames, ...
                "ShowLabels", obj.ShowBeamFrames, ...
                "drawCrossSections", true, ...
                "interpolateBeam", true, ...
                "FaceAlpha", 0.3, ...
                "Color",obj.Color, ...
                "Height", obj.linkDef.beamPars.H, ...
                "Width", obj.linkDef.beamPars.W ...
                );

            % Draw Joint and TCP frame
            obj = drawJoint(obj, obj.beamVis.transfCS([1,end]));
            obj = obj.drawTCPFrame(obj.beamVis.transfCS(end));

            % Links from/to joints
            g_J1_B_inv = inv(obj.linkDef.g_J_B);
            obj.flexLinkJointPlot = plot3( ...
                [g_J1_B_inv(1,4), 0], ...
                [g_J1_B_inv(2,4), 0], ...
                [g_J1_B_inv(3,4), 0], ...
                "Color", obj.Color, ...
                "LineWidth",3, ...
                "Parent", obj.beamVis.transfCS(1) ...
                );

            % Tendons
            nCables = length(obj.linkDef.tendonActuation.x_td_funs);
            if nCables
                colorsTendons = lines(nCables);
                lBeam = obj.linkDef.L/obj.linkDef.nSeg;
                sBeamNodes = 0:lBeam:obj.linkDef.L;
                [g_cm, termNodes] = obj.linkDef.tendonActuation.getNodeData( ...
                    sBeamNodes);

                for iC = 1:nCables
                    g_c = zeros(4, 4, termNodes(iC));

                    for iN = 1:termNodes(iC)
                        % Cable points initial configuration
                        g_c(:,:,iN) = g(:,:,iN) * g_cm(:,:,iN,iC);
                    end

                    obj.tendonVis(iC) = elara.visualization.elasticBeam( g_c, ...
                        "edgeAlpha", 0, ...
                        "FaceAlpha", 0, ...
                        "DrawEdges", false, ...
                        "drawCenterline", true, ...
                        "drawCrossSections", true, ...
                        "showFrames", false, ...
                        "interpolateBeam", true, ...
                        "Color", colorsTendons(iC,:), ...
                        "Height", 0.003, "Width", 0.003, ...
                        "ShowFrames", false, ...
                        "Visible", obj.ShowTendons...
                        );
                end
            end
        end
    end
    %% Update methods for property changes
    methods(Access = private)
        function obj = onColorChanged(obj)
            obj.cSysJ.AxisColors        = repmat(obj.Color.', [3,1]);
            obj.cSysTCP.AxisColors      = repmat(obj.Color.', [3,1]);            
            obj.flexLinkJointPlot.Color = obj.Color;
            obj.jointPatch.FaceColor    = obj.Color;
            obj.jointPatch.EdgeColor    = obj.Color;
            obj.beamVis.Color           = obj.Color;
        end
        function obj = onNameChanged(obj)
        end
        function obj = onShowJointChanged(obj)
        end
        function obj = onShowTCPFrameChanged(obj)
            obj.cSysTCP.Visible = obj.linkDef.hasTCP && obj.ShowTCPFrame;
        end
        function obj = onShowTendonsChanged(obj)
        end
        function obj = onShowBeamFramesChanged(obj)
        end
    end
end