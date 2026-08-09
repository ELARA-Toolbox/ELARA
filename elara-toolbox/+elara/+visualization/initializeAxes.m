function [figureHandle, axesHandle] = initializeAxes(options, figureArguments)
    %INITIALIZEAXES Initialize axes for three-dimensional Cartesian plots.
    %   [FIG, AX] = ELARA.VISUALIZATION.INITIALIZEAXES creates a figure and
    %   axes, enables the grid and hold state, uses equal scaling, and sets
    %   a three-dimensional view. Figure name-value arguments are forwarded
    %   to FIGURE.
    %
    %   ELARA.VISUALIZATION.INITIALIZEAXES("createFigure", false) applies
    %   the settings to the current axes instead of creating a new figure.
    arguments
        options.createFigure (1,1) {mustBeNumericOrLogical} = true
        figureArguments.?matlab.ui.Figure
    end

    if options.createFigure
        figureArgumentsCell = namedargs2cell(figureArguments);
        figureHandle = figure(figureArgumentsCell{:});
        axesHandle = axes(figureHandle);
    else
        figureHandle = gcf;
        axesHandle = gca;
    end

    grid(axesHandle, "on");
    hold(axesHandle, "on");
    axis(axesHandle, "equal");
    view(axesHandle, [37.5, 30]);
    xlabel(axesHandle, "x axis");
    ylabel(axesHandle, "y axis");
    zlabel(axesHandle, "z axis");
end
