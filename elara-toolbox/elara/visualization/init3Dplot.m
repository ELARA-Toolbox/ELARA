function [fh, ax] = init3Dplot(options, figureArgs)
    % INIT3DPLOT Initialize a figure window for 3D cartesian plots
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % Department of Mechanical Engineering
    % Technical University of Munich
    %
    % By default, the function creates a new figure. To not create a new
    % figure, call it with "createFigure", 0.
    % The other name-value pairs are given as arguments to the new figure 
    % (if created).
    
    arguments
        % Optional arguments
        % Create new figure on function call (or just modify existing)
        options.createFigure (1,1) {mustBeNumericOrLogical} = 1;
        
        % Name-Value-Pair arguments that are passed to the newly
        % created figure (only applicable if createFigure = 1)
        figureArgs.?matlab.ui.Figure
    end

    % Create new figure if required
    if options.createFigure
        figureArgsCell = namedargs2cell(figureArgs);
        fh = figure(figureArgsCell{:});
        ax = axes(fh);
    else
        fh = gcf;
    end
    
    % Old solution for arguments check
    % if (nargin == 0) || (nargin > 0 && ~strcmp(varargin{1}, 'nofigure'))
    %     fh = figure(varargin{:});
    % else
    %     fh = gcf;
    % end
    
    % Settings
    grid on; hold on
    axis equal
    view([37.5, 30])
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
end

