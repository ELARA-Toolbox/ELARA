classdef coordSysSE3 < handle
    %% Visualization of a 3D (cartesian) coordinate frame
    % with the pose by a SE3 matrix
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Transform object that specifies the frame's configuration
        transf  (1,1) matlab.graphics.primitive.Transform

        % text label handles
        h_coordAxes   (3,1) matlab.graphics.chart.primitive.Quiver
        h_axisLabels  (3,1) matlab.graphics.primitive.Text
        h_nameLabel   (1,1) matlab.graphics.primitive.Text
    end
    properties (SetObservable)
        % scale, name and color
        Scale       (1,1) double = 1;
        Name        (1,1) string = "0";

        % Colors: One row of RGB values for each axis
        AxisColors  (3,3) double = lines(3);

        % texts
        axisLabels    (3,1) string = ["$x$"; "$y$"; "$z$"];

        % Visibility of the overall coordinate frame
        Visible         (1,1) matlab.lang.OnOffSwitchState = true;

        % Other settings
        DrawLabels      (1,1) matlab.lang.OnOffSwitchState = true;
        LabelFontSize   (1,1) double  = 10
        MaxHeadSize     (1,1) double  = 1.5
        LineWidth       (1,1) double  = 1.5
    end
    %% Main methods
    methods
        function obj = coordSysSE3(g, options)
            % Construct coordinate system
            arguments
                % SE3 matrix that defines the system's pose
                % (orientation and position)
                g (4, 4) {mustBeNumeric} = eye(4);

                % Optional arguments
                options.Parent         (1,1) = gca;
                options.Scale          (1,1) {mustBeNumeric} = 0.1
                options.Name           (1,:) char            = '0'
                options.DrawLabels     (1,1) logical         = 1
                options.LabelFontSize  (1,1) {mustBeNumeric} = 10
                options.MaxHeadSize    (1,1) {mustBeNumeric} = 1.5
                options.LineWidth      (1,1) {mustBeNumeric} = 1.5
                options.AxisColors     (3,3) {mustBeNumeric} = lines(3)
                options.Visible        (1,1) logical = true
            end

            obj.Scale   = options.Scale;
            obj.Name    = options.Name;
            obj.DrawLabels     = options.DrawLabels;
            obj.LabelFontSize  = options.LabelFontSize;
            obj.MaxHeadSize    = options.MaxHeadSize;
            obj.LineWidth      = options.LineWidth;
            obj.AxisColors     = options.AxisColors;
            obj.Visible        = options.Visible;

            obj = initCoordSys(obj, options.Parent);
            obj.transf.Matrix = g;

            % Register listeners to process property changes
            addlistener(obj,'Visible','PostSet', @(src,evt)obj.onVisibleChanged);
            addlistener(obj,'AxisColors','PostSet', @(src,evt)obj.onColorChanged);
            addlistener(obj,'Name','PostSet', @(src,evt)obj.onNameChanged);
            addlistener(obj,'axisLabels','PostSet', @(src,evt)obj.onAxisLabelsChanged);
            addlistener(obj,'MaxHeadSize','PostSet', @(src,evt)obj.onMaxHeadSizeChanged);
            addlistener(obj,'LabelFontSize','PostSet', @(src,evt)obj.onLabelFontSizeChanged);
            addlistener(obj,'LineWidth','PostSet', @(src,evt)obj.onLineWidthChanged);
            addlistener(obj,'DrawLabels','PostSet', @(src,evt)obj.onDrawLabelsChanged);
            addlistener(obj,'Scale','PostSet', @(src,evt)obj.onScaleChanged);
        end

        function obj = initCoordSys(obj, parent)
            % Draw cartesian coordinate system at point p, with rotation
            % characterized by rotation matrix R = R_IB (I: inertial frame)

            obj.transf = hgtransform(parent);

            % Get plot colors
            if size(obj.AxisColors,1) == 1
                axisColors = repmat(obj.AxisColors,[3,1]);
            else
                axisColors = obj.AxisColors;
            end

            % Create axis arrows
            obj.h_coordAxes(1) = quiver3( 0,0,0, obj.Scale,0,0, ...
                'LineWidth', obj.LineWidth, ...
                "MaxHeadSize", obj.MaxHeadSize, ...
                'Color', axisColors(1, :), ...
                "Parent", obj.transf, ...
                "Visible", obj.Visible ...
                );

            obj.h_coordAxes(2) = quiver3( 0,0,0, 0,obj.Scale,0, ...
                'LineWidth', obj.LineWidth, ...
                "MaxHeadSize", obj.MaxHeadSize, ...
                'Color', axisColors(2, :), ...
                "Parent", obj.transf, ...
                "Visible", obj.Visible ...
                );

            obj.h_coordAxes(3) = quiver3( 0,0,0, 0,0,obj.Scale, ...
                'LineWidth', obj.LineWidth, ...
                "MaxHeadSize", obj.MaxHeadSize, ...
                'Color', axisColors(3, :), ...
                "Parent", obj.transf, ...
                "Visible", obj.Visible ...
                );


            % Create axis labels
            obj.h_axisLabels(1) = text( obj.Scale,0,0, ...
                strcat("", obj.axisLabels(1), ""), ...
                "interpreter", "latex", ...
                "FontSize", obj.LabelFontSize, ...
                "Parent", obj.transf, ...
                "Visible", obj.DrawLabels && obj.Visible ...
                );

            obj.h_axisLabels(2) = text( 0,obj.Scale,0, ...
                strcat("", obj.axisLabels(2), ""), ...
                "interpreter", "latex", ...
                "FontSize", obj.LabelFontSize, ...
                "Parent", obj.transf, ...
                "Visible", obj.DrawLabels && obj.Visible ...
                );

            obj.h_axisLabels(3) = text( 0,0,obj.Scale, ...
                strcat("", obj.axisLabels(3), ""), ...
                "interpreter", "latex", ...
                "FontSize", obj.LabelFontSize, ...
                "Parent", obj.transf, ...
                "Visible", obj.DrawLabels && obj.Visible ...
                );

            offset_scale = 0.25;
            obj.h_nameLabel = text( ...
                +obj.Scale*offset_scale, ...
                0, ...
                -obj.Scale*offset_scale, ...
                strcat("", obj.Name, ""), ...
                "interpreter", "latex", ...
                "FontSize", obj.LabelFontSize, ...
                "Parent", obj.transf, ...
                "Visible", obj.DrawLabels && obj.Visible ...
                );
        end
    end

    %% Update methods for property changes
    methods(Access = private)
        function obj = onVisibleChanged(obj)
            obj.h_nameLabel.Visible = obj.Visible && obj.DrawLabels;
            for iAxis = 1:3
                obj.h_coordAxes(iAxis).Visible  = obj.Visible;
                obj.h_axisLabels(iAxis).Visible = obj.Visible && obj.DrawLabels;
            end
        end
        function obj = onColorChanged(obj)
            obj.h_coordAxes(1).Color = obj.AxisColors(1, :);
            obj.h_coordAxes(2).Color = obj.AxisColors(2, :);
            obj.h_coordAxes(3).Color = obj.AxisColors(3, :);
        end
        function obj = onNameChanged(obj)
            obj.h_nameLabel.String = obj.Name;
        end
        function obj = onAxisLabelsChanged(obj)
            obj.h_axisLabels(1).String = strcat("", obj.axisLabels(1), "");
            obj.h_axisLabels(2).String = strcat("", obj.axisLabels(2), "");
            obj.h_axisLabels(3).String = strcat("", obj.axisLabels(3), "");
        end
        function obj = onMaxHeadSizeChanged(obj)
            for iAxis = 1:3
                obj.h_coordAxes(iAxis).MaxHeadSize = obj.MaxHeadSize;
            end
        end
        function obj = onLabelFontSizeChanged(obj)
            obj.h_nameLabel.FontSize = obj.LabelFontSize;
            for iAxis = 1:3
                obj.h_axisLabels(iAxis).FontSize = obj.LabelFontSize;
            end
        end
        function obj = onLineWidthChanged(obj)
            for iAxis = 1:3
                obj.h_coordAxes(iAxis).LineWidth = obj.LineWidth;
            end
        end
        function obj = onDrawLabelsChanged(obj)
            obj.h_nameLabel.Visible = obj.Visible && obj.DrawLabels;
            for iAxis = 1:3
                obj.h_coordAxes(iAxis).Visible  = obj.Visible;
                obj.h_axisLabels(iAxis).Visible = obj.Visible && obj.DrawLabels;
            end
        end
        function obj = onScaleChanged(obj)
            obj.h_coordAxes(1).UData = obj.Scale;
            obj.h_coordAxes(2).VData = obj.Scale;
            obj.h_coordAxes(3).WData = obj.Scale;
            for iAxis = 1:3
                obj.h_axisLabels(iAxis).Position(iAxis) = obj.Scale;
            end
        end
    end
end

