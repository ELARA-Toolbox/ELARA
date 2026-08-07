classdef SystemVisualization < handle
    %% Class for the Visualization of a complete Multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        system              (1,1) elara.SystemNum

        % Array of link visualization systemVisualizationects
        linkVisualization   (1,:) elara.abstract.LinkVisualization
    end
    properties(SetObservable)
        showInertialFrame   (1,1) logical = true;
        showLinkFrames      (1,1) logical = true;
        showBeamFrames      (1,1) logical = false;
        showJoints          (1,1) logical = true;
        showTendons         (1,1) logical = true;
    end
    properties(SetAccess=protected)
        % Coordinate frame for the inertial system
        coordSysI               (1,1) CoordSysSE3
    end

    %% Main Methods
    methods
        function systemVisualization = SystemVisualization(system, links, g, options)
            % Construct an instance of this class
            arguments
                system   (1,1) elara.abstract.System

                links   (1,:) elara.abstract.Link = elara.RigidLink.empty();

                % Array of SE3 matrices defining the system's configuration
                g       (4,4,:) double {elara.internal.validation.mustBeSE3Matrix} = zeros(4,4,0)

                % Additional options
                options.showInertialFrame  (1,1) logical = true;
                options.showLinkFrames     (1,1) logical = true;
                options.showBeamFrames     (1,1) logical = false;
                options.showJoints         (1,1) logical = true;
                options.showTendons        (1,1) logical = true;

                % Function handle for a color map used to specify link 
                % colors in the form colors = cmap(n), where n is the nr.
                % of link
                options.linkColorMap       (1,1) function_handle = @lines;
            end

            % Assign properties
            systemVisualization.system             = system;
            systemVisualization.showInertialFrame = options.showInertialFrame;
            systemVisualization.showLinkFrames    = options.showLinkFrames;
            systemVisualization.showBeamFrames    = options.showBeamFrames;
            systemVisualization.showJoints        = options.showJoints;
            systemVisualization.showTendons       = options.showTendons;

            systemVisualization = initializeVisualization(systemVisualization, links, options.linkColorMap);

            if ~isempty(g)
                % If configuration is given: Draw given configuration
                systemVisualization = systemVisualization.updateConfiguration(g);
            else
                % If no configuration is given: Draw reference config.
                systemVisualization = systemVisualization.visualizeReferenceConfig;
            end

            % TODO: Implement property listeners to update graphics objects
            % when properties are changed
        end

        function systemVisualization = initializeVisualization(systemVisualization, links, linkColorMap)
            %% Initialize drawing for the full system
            arguments
                systemVisualization     (1,1) elara.visualization.SystemVisualization
                links   (:,1) elara.abstract.Link

                % Color map used to specify link colors
                linkColorMap (1,1) function_handle
            end

            % Draw inertial frame
            coordSysScale = 0.075;
            inertialColor = summer(1);
            systemVisualization.coordSysI = CoordSysSE3(eye(4), ...
                'scale', coordSysScale, ...
                'name','I', ...
                'AxisColors', repmat(inertialColor, [3,1]), ...
                "Visible", systemVisualization.showInertialFrame);

            % Initialize links
            linkColors = linkColorMap(systemVisualization.system.nLinks);
            for iLink = 1:numel(links)
                systemVisualization.linkVisualization(iLink) = links(iLink).getLinkVisualization( ...
                    "showJoint", systemVisualization.showJoints, ...
                    "Name", num2str(iLink), ...
                    "Color", linkColors(iLink,:) ...
                    );

                if links(iLink).isRigid
                    systemVisualization.linkVisualization(iLink).showLinkFrame = systemVisualization.showLinkFrames;
                else
                    systemVisualization.linkVisualization(iLink).showTendons = systemVisualization.showTendons;
                    systemVisualization.linkVisualization(iLink).showBeamFrames = systemVisualization.showBeamFrames;
                end
            end
        end

        function systemVisualization = visualizeReferenceConfig(systemVisualization)
            %% Visualize the reference configuration of the given system
            q = zeros(1,systemVisualization.system.nDoF);
            gRef = systemVisualization.system.computeFwdKin(q);
            systemVisualization = systemVisualization.updateConfiguration(gRef);
        end

        function systemVisualization = updateConfiguration(systemVisualization, g)
            arguments
                systemVisualization (1,1)
                g   (4,4,:) double {elara.internal.validation.mustBeSE3Matrix}
            end
            for iLink = 1:systemVisualization.system.nLinks
                % Add fixed node to configuration array if the beam is a
                % cantilever beam
                gLink = g(:,:,systemVisualization.system.linkFrameIndices(1,iLink):systemVisualization.system.linkFrameIndices(2,iLink));
                if iLink == 1 && systemVisualization.system.isCantilever
                    g0 = systemVisualization.system.g0;% * systemVisualization.system.linkData.g_J1_B(:,:,iLink);
                    gLink = cat(3, g0, gLink);
                end
                systemVisualization.linkVisualization(iLink) = systemVisualization.linkVisualization(iLink).updateConfiguration(gLink);
            end
        end

        function systemVisualization = animateSystem(systemVisualization, simRes, fig, options)
            arguments
                systemVisualization

                % SimResults object containing the simulation data
                simRes          (1,1) elara.SimulationResults

                % Figure handle to the parent figure
                fig

                options.frameRate  (1,1) double = 30;

                % Save animation to file?
                options.saveMovie  (1,1) logical = false;

                % File name for the saved movie (full path)
                options.fileName   (1,1) string = "animation";
            end


            %% Interpolate results at fixed sampling rate
            tSample = 1/options.frameRate;
            tQuery = simRes.tout(1):tSample:simRes.tout(end);
            gQuery = elara.internal.simulation.interpolateResults( ...
                systemVisualization.system, simRes, tQuery);


            %% Prepare animation

            % Add text for current time step
            textTime = text(0,0,0.3,'', 'Interpreter', 'latex');

            if ~strcmp(fig.WindowStyle, 'docked')
                fig.WindowState = 'maximized';
            end

            %% Prepare axis limits
            % Extra margin
            margins = [-1, -1, -1; 1, 1, 1]*0.1;

            % Set limits
            [xlimits, ylimits, zlimits] = getAxisLimits(systemVisualization, gQuery, margins);
            xlim(xlimits);
            ylim(ylimits);
            zlim(zlimits);

            %% Prepare video capture
            % for better performance

            if options.saveMovie
                % Get frame dimensions
                systemVisualization = systemVisualization.updateConfiguration(gQuery(:,:,:,1));
                testFrame = getframe(fig);

                % Initialize array
                % Preallocate animation frames struct
                % (taken from getframe documentation)
                animFrame= struct( ...
                    'cdata', repmat({zeros(size(testFrame.cdata))}, length(tQuery), 1), ...
                    'colormap', repmat({zeros(size(testFrame.colormap))}, length(tQuery), 1) ...
                    );
            end

            %% Animate frames
            for iStep = 1:length(tQuery)
                % Check if figure is closed
                if ~isvalid(fig)
                    return
                else
                    textTime.String = sprintf('$t$ = %.3fs', tQuery(iStep));

                    systemVisualization = systemVisualization.updateConfiguration(gQuery(:,:,:,iStep));

                    if ~options.saveMovie
                        % Drawnow is only necessary if we don't save the
                        % animation to a movie since getframe triggers
                        % drawnow
                        drawnow;
                        drawnow;
                    end
                    if isvalid(fig) && options.saveMovie
                        try
                            animFrame(iStep) = getframe(fig);
                        catch
                            warning("Failed to get video frame.")
                            return;
                        end
                    end
                end
            end

            %% Write to video
            % (code from MATLAB docs)
            if options.saveMovie
                disp('Saving as Video...')

                % Check whether all video frames have the same size
                firstDimension = size(animFrame(1).cdata);
                sizesEqual = true;
                for iFrame = 1:length(animFrame)
                    if ~all(size(animFrame(iFrame).cdata) == firstDimension)
                        sizesEqual = false;
                        warning("Could not save video to file since the video frame size has changed during the animation. The size of the animation figure must be kept constant.");
                        break;
                    end
                end

                % Write actual video
                if sizesEqual
                    v = VideoWriter(options.fileName, 'MPEG-4');
                    v.Quality = 100;
                    v.FrameRate = options.frameRate;
                    v.Quality = 95;
                    open(v);
                    for iFrame = 1:length(animFrame)
                        writeVideo(v,animFrame(iFrame));
                    end
                    close(v);
                end
            end
        end


    end
    %% Helper methods
    methods(Hidden)
        function [xlimits, ylimits, zlimits] = getAxisLimits(systemVisualization, g, margins)
            %% Compute axis limits for simulation results
            arguments
                systemVisualization     (1,1)

                % Array of SE3 configuration matrices for simulation results
                % dimensions (4,4,nFrames,nSteps)
                g       (4,4,:,:) double

                % Additional margins in positive and negative direction
                % added to each axis;
                % in the columns are the pos. and neg. values for x,y,z axes
                margins  (2,3) double
            end

            % Add fixed node to configuration array if the system is a
            % cantilever beam
            if systemVisualization.system.isCantilever
                x0 = systemVisualization.system.g0(1:3,4);
            else
                x0 = nan(3,1);
            end

            xlimits = [
                min(min(g(1,4,:,:),[], 'all'), x0(1))
                max(max(g(1,4,:,:),[], 'all'), x0(1))
                ] + margins(:,1);
            ylimits = [
                min(min(g(2,4,:,:),[], 'all'), x0(2))
                max(max(g(2,4,:,:),[], 'all'), x0(2))
                ] + margins(:,2);
            zlimits = [
                min(min(g(3,4,:,:),[], 'all'), x0(3))
                max(max(max(g(3,4,:,:),[], 'all'), 0.3), x0(3))
                ] + margins(:,3); % Consider position of the time label
        end
    end
end
