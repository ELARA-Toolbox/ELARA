classdef ElasticBeam < handle
    % ElasticBeam Class to draw an elastic beam in a 3D plot.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    %
    % Visualization of a spatially discretized.
    % Note: Direction of the beam length is in (local) Z-direction, i.e.,
    % the cross-sections lie in the XY planes of the local frames
    % (corresponding to [Dem12] / [Dem+15]).
    %
    % Properties and methods are mostly self-explanatory.

    properties (SetObservable)
        %% Beam properties
        % Beam width and height
        width   (1,1) {mustBeNumeric} = 0.05;
        height  (1,1) {mustBeNumeric} = 0.05;

        % Beam color
        Color               (3,1) double = lines(1);

        % Enable / Disable coordinate system label
        ShowLabels          (1,1) matlab.lang.OnOffSwitchState = false;

        % Enable / Disable coordinate systems
        ShowFrames          (1,1) matlab.lang.OnOffSwitchState = true;

        % Plot center line?
        DrawCenterline      (1,1) matlab.lang.OnOffSwitchState = false;

        % Draw cross sections?
        DrawCrossSections   (1,1) matlab.lang.OnOffSwitchState = true;

        % Draw beam edges
        DrawEdges           (1,1) matlab.lang.OnOffSwitchState = true;

        % Opacity of the beam edges
        EdgeAlpha           (1,1) double = 1;

        % Opacity of the beam faces
        FaceAlpha           (1,1) double = 0.2;

        % Interpolate the center line and beam edges?
        InterpolateBeam     (1,1) matlab.lang.OnOffSwitchState = true;

        % (Normalized) segment length for spatial interpolation
        lInterp             (1,1) double = 0.01;

        % Overall visibility of the object
        Visible             (1,1) matlab.lang.OnOffSwitchState = true;
    end
    properties(SetAccess = protected)
        %% Graphics objects

        % Transform objects that describe the absolute configurations of
        % the individual cross sections
        crossSectionTransforms        (:,1) matlab.graphics.primitive.Transform

        % Center line plot3 object
        centerLine     (1,1) matlab.graphics.chart.primitive.Line

        % Array with node coordinate system objects
        crossSectionCoordSys           (:,1) CoordSysSE3

        % End coordinate system object
        endCoordSys        (1,1) CoordSysSE3

        % Array with patches for cross-sections
        crossSectionPatches    (:,1) matlab.graphics.primitive.Patch

        % Patch object for beam faces/edges
        beamPatch      (1,1) matlab.graphics.primitive.Patch
    end

    methods
        function obj = ElasticBeam(g, options)
            %% Constructor
            arguments
                % Array of SE3 node configuration matrices with dimension
                % (4, 4, nNodes)
                g (4,4,:) double = zeros(4,4,0)

                % Class properties as optional arguments
                options.?elara.visualization.ElasticBeam;
            end

            % Set class properties from optional arguments
            optFields = fields(options);
            for iOpt = 1:length(optFields)
                obj.(optFields{iOpt}) = options.(optFields{iOpt});
            end

            if ~isempty(g)
                obj = obj.drawBeam(size(g,3));
                obj = obj.updateConfiguration(g);
            end

            % Register listeners to process property changes
            addlistener(obj,'Visible','PostSet', @(src,evt)obj.onVisibleChanged);
            addlistener(obj,'Color','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'ShowFrames','PostSet', @(src,evt)obj.onShowFramesChanged);
            addlistener(obj,'ShowLabels','PostSet', @(src,evt)obj.onShowLabelsChanged);
            addlistener(obj,'DrawEdges','PostSet', @(src,evt)obj.onDrawEdgesChanged);
            addlistener(obj,'DrawCenterline','PostSet', @(src,evt)obj.onDrawCenterlineChanged);
            addlistener(obj,'DrawCrossSections','PostSet', @(src,evt)obj.onDrawCrossSectionsChanged);
        end

        function obj = drawBeam(obj, nNodes)
            %% Initialize the beam drawing

            % Make sure hold is enabled to avoid problems with deleted graphics objects
            hold on;

            % Matrix with corners in local frames
            % Vectors are in the rows; cross-sections are in the XY-plane
            % Order: Top left, top right, bottom right, bottom left
            cornersLocal = [
                -obj.width, +obj.height, 0
                -obj.width, -obj.height, 0
                +obj.width, -obj.height, 0
                +obj.width, +obj.height , 0
                ]*0.5;

            for iN = 1:nNodes
                % Initialize transform object for the node
                obj.crossSectionTransforms(iN) = hgtransform();

                % Plot node coordinate frame
                obj.crossSectionCoordSys(iN) = CoordSysSE3(...
                    eye(4), ...
                    "DrawLabels", obj.ShowLabels, ...
                    'Scale', 0.03, 'Name', num2str(iN),...
                    "Parent", obj.crossSectionTransforms(iN), ...
                    "Visible", obj.ShowFrames && obj.Visible...
                    );

                % Plot cross-section
                obj.crossSectionPatches(iN) = patch( ...
                    cornersLocal(:,1), cornersLocal(:,2), cornersLocal(:,3), [0,0,0], ...
                    "FaceColor", obj.Color, ...
                    "EdgeColor", obj.Color, ...
                    "FaceAlpha", 0.0, ...
                    "Parent", obj.crossSectionTransforms(iN), ...
                    "Visible", obj.DrawCrossSections && obj.Visible ...
                    );
            end

            % Plot center line
            obj.centerLine= plot3(...
                zeros(1,nNodes), zeros(1,nNodes), zeros(1,nNodes), ...
                "Color", obj.Color,"LineWidth", 1, ...
                "Visible", obj.DrawCenterline && obj.Visible...
                );

            % Draw beam edges/faces
            if obj.InterpolateBeam
                nNodesInterp = (1/obj.lInterp)+1;
            else
                nNodesInterp = nNodes;
            end

            % Matrix with vertex indices for the edges
            % Rows: Edge #, Columns: Vertex indices for a row
            vertexNrs = reshape(1:(4*nNodesInterp),nNodesInterp, 4).';

            % Matrix with vertex indies for the faces
            % Rows: Face #, Columns: Vertex indices for a face
            Faces = [
                vertexNrs(1,:), flip(vertexNrs(2,:))
                vertexNrs(2,:), flip(vertexNrs(3,:))
                vertexNrs(3,:), flip(vertexNrs(4,:))
                vertexNrs(4,:), flip(vertexNrs(1,:))
                ];

            % Matrix of all vertices (in the rows)
            vert = zeros(4*nNodesInterp,3);

            obj.beamPatch = patch( ...
                "Faces", Faces, "Vertices", vert, ...
                "FaceColor", obj.Color, "EdgeColor", obj.Color, ...
                "FaceAlpha", obj.FaceAlpha, ...
                "Visible", obj.Visible && obj.DrawEdges);
        end

        function obj = updateConfiguration(obj, g)
            % Update beam configuration
            arguments
                obj

                % Array of SE3 node configuration matrices with dimension
                % (4, 4, nNodes)
                g (4,4,:) double
            end
            nNodes = size(g,3);

            if isempty(obj.crossSectionTransforms) || nNodes ~= length(obj.crossSectionTransforms)
                obj = obj.drawBeam(nNodes);
            end

            % Update CS transforms
            for iN = 1:nNodes
                obj.crossSectionTransforms(iN).Matrix = g(:,:,iN);
            end

            % Interpolate data if needed (spatial interpolation on SE3)
            if obj.InterpolateBeam && (obj.DrawCenterline || obj.EdgeAlpha || obj.FaceAlpha)
                % Spatial query points (normalize beam length to 1)
                sInput = 0:1/(nNodes-1):1;
                sQuery = 0:obj.lInterp:1;

                gInterp = elara.SE3.interpolateConfigurations(g, sInput, sQuery);
            else
                gInterp = g;
            end

            % Update center line
            obj.centerLine.XData = gInterp(1,4,:);
            obj.centerLine.YData = gInterp(2,4,:);
            obj.centerLine.ZData = gInterp(3,4,:);

            % Update beam edges/faces
            % Get vertex positions for new configuration
            obj.beamPatch.Vertices = ...
                reshape( obj.computeAbsoluteCSCorners(gInterp), 3, []).';
        end
    end

    %% Update methods for property changes
    methods(Access = private)
        function obj = onVisibleChanged(obj)
            for iN = 1:length(obj.crossSectionCoordSys)
                obj.crossSectionCoordSys(iN).Visible = obj.ShowFrames && obj.Visible;
                obj.crossSectionPatches(iN).Visible = obj.DrawCrossSections && obj.Visible;
            end
            obj.centerLine.Visible = obj.DrawCenterline && obj.Visible;
            obj.beamPatch.Visible = obj.Visible && obj.DrawEdges;
        end
        function obj = onColorChanged(obj)
            % ToDo: Also CS frame colors?
            obj.centerLine.Color    = obj.Color;
            obj.beamPatch.FaceColor = obj.Color;
            obj.beamPatch.EdgeColor = obj.Color;
        end
        function obj = onShowFramesChanged(obj)
            for iN = 1:length(obj.crossSectionCoordSys)
                obj.crossSectionCoordSys(iN).Visible = obj.ShowFrames && obj.Visible;
            end
        end
        function obj = onShowLabelsChanged(obj)
            for iN = 1:length(obj.crossSectionCoordSys)
                obj.crossSectionCoordSys(iN).DrawLabels = obj.ShowLabels;
            end
        end
        function obj = onDrawEdgesChanged(obj)
            obj.beamPatch.Visible = obj.Visible && obj.DrawEdges;
        end
        function obj = onDrawCenterlineChanged(obj)
            obj.centerLine.Visible = obj.DrawCenterline && obj.Visible;
        end
        function obj = onDrawCrossSectionsChanged(obj)
            for iN = 1:length(obj.crossSectionCoordSys)
                obj.crossSectionPatches(iN).Visible = obj.DrawCrossSections && obj.Visible;
            end
        end
    end
    %% Protected methods
    methods(Access=protected)
        function cornersAbs = computeAbsoluteCSCorners(obj, g)
            % Computes the coordinates of the cross-section corners in
            % global frame

            nNodes = size(g,3);

            % Matrix with corners in local frames
            % Vectors are in the rows; cross-sections are in the XY-plane
            % Order: Top left, top right, bottom right, bottom left
            cornersLocal = [
                -obj.width, +obj.height, 0
                -obj.width, -obj.height, 0
                +obj.width, -obj.height, 0
                +obj.width, +obj.height, 0
                ].'*0.5;

            % Written as homogeneous points with added 1
            cornersLocal = [cornersLocal; ones(1,4)];

            % Compute absolute points
            cornersAbs = zeros(3,nNodes,4);
            for iNode = 1:nNodes
                % Compute matrix with corner vectors in global frame
                % (Vectors in columns; makes computation easier)
                cornersAbsNode = g(:,:,iNode) * cornersLocal;
                cornersAbs(:,iNode,:) = cornersAbsNode(1:3,:);
            end
        end
    end
end
