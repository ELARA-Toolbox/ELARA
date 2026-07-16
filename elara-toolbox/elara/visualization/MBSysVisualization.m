classdef MBSysVisualization < handle
    %% Class for the Visualization of a complete Multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        system              (1,1) elara.SystemNum

        % Array of link visualization objects
        linkVis             (1,:) MBLinkVisualization
    end
    properties(SetObservable)
        ShowInertialFrame   (1,1) logical = true;
        ShowLinkFrames      (1,1) logical = true;
        ShowBeamFrames      (1,1) logical = false;
        ShowJoints          (1,1) logical = true;
        ShowTendons         (1,1) logical = true;
    end
    properties(SetAccess=protected)
        % Coordinate frame for the inertial system
        cSysI               (1,1) coordSysSE3
    end

    %% Main Methods
    methods
        function obj = MBSysVisualization(MBSys, links, g, opts)
            % Construct an instance of this class
            arguments
                MBSys   (1,1) elara.internal.System

                links   (1,:) elara.internal.Link = elara.RigidLink.empty();

                % Array of SE3 matrices defining the system's configuration
                g       (4,4,:) double {mustBeSE3MatrixArray} = zeros(4,4,0)

                % Additional options
                opts.ShowInertialFrame  (1,1) logical = true;
                opts.ShowLinkFrames     (1,1) logical = true;
                opts.ShowBeamFrames     (1,1) logical = false;
                opts.ShowJoints         (1,1) logical = true;
                opts.ShowTendons        (1,1) logical = true;

                % Function handle for a color map used to specify link 
                % colors in the form colors = cmap(n), where n is the nr.
                % of link
                opts.linkColorMap       (1,1) function_handle = @lines;
            end

            % Assign properties
            obj.system             = MBSys;
            obj.ShowInertialFrame = opts.ShowInertialFrame;
            obj.ShowLinkFrames    = opts.ShowLinkFrames;
            obj.ShowBeamFrames    = opts.ShowBeamFrames;
            obj.ShowJoints        = opts.ShowJoints;
            obj.ShowTendons       = opts.ShowTendons;

            obj = initializeVisualization(obj, links, opts.linkColorMap);

            if ~isempty(g)
                % If configuration is given: Draw given configuration
                obj = obj.updateConfiguration(g);
            else
                % If no configuration is given: Draw reference config.
                obj = obj.visualizeReferenceConfig;
            end

            % TODO: Implement property listeners to update graphics objects
            % when properties are changed
        end

        function obj = initializeVisualization(obj, links, linkColorMap)
            %% Initialize drawing for the full system
            arguments
                obj     (1,1) MBSysVisualization
                links   (:,1) elara.internal.Link

                % Color map used to specify link colors
                linkColorMap (1,1) function_handle
            end

            % Draw inertial frame
            coordSysScale = 0.075;
            inertialColor = summer(1);
            obj.cSysI = coordSysSE3(eye(4), ...
                'scale', coordSysScale, ...
                'name','I', ...
                'AxisColors', repmat(inertialColor, [3,1]), ...
                "Visible", obj.ShowInertialFrame);

            % Initialize links
            linkColors = linkColorMap(obj.system.nLinks);
            for iLink = 1:numel(links)
                obj.linkVis(iLink) = links(iLink).getLinkVisualization( ...
                    "ShowJoint", obj.ShowJoints, ...
                    "Name", num2str(iLink), ...
                    "Color", linkColors(iLink,:) ...
                    );

                if links(iLink).isRigid
                    obj.linkVis(iLink).ShowLinkFrame = obj.ShowLinkFrames;
                else
                    obj.linkVis(iLink).ShowTendons = obj.ShowTendons;
                    obj.linkVis(iLink).ShowBeamFrames = obj.ShowBeamFrames;
                end
            end
        end

        function obj = visualizeReferenceConfig(obj)
            %% Visualize the reference configuration of the given system
            q = zeros(1,obj.system.nDoF);
            gRef = obj.system.computeFwdKin(q);
            obj = obj.updateConfiguration(gRef);
        end

        function obj = updateConfiguration(obj, g)
            arguments
                obj (1,1)
                g   (4,4,:) double {mustBeSE3MatrixArray}
            end
            for iLink = 1:obj.system.nLinks
                % Add fixed node to configuration array if the beam is a
                % cantilever beam
                gLink = g(:,:,obj.system.linkFrameIndices(1,iLink):obj.system.linkFrameIndices(2,iLink));
                if iLink == 1 && obj.system.isCantilever
                    g0 = obj.system.g0;% * obj.system.linkData.g_J1_B(:,:,iLink);
                    gLink = cat(3, g0, gLink);
                end
                obj.linkVis(iLink) = obj.linkVis(iLink).updateConfiguration(gLink);
            end
        end

        function obj = animateSystem(obj, simRes, fig, opts)
            arguments
                obj

                % SimResults object containing the simulation data
                simRes          (1,1) MBSimResults

                % Figure handle to the parent figure
                fig

                opts.frameRate  (1,1) double = 30;

                % Save animation to file?
                opts.saveMovie  (1,1) logical = false;

                % File name for the saved movie (full path)
                opts.fileName   (1,1) string = "animation";
            end


            %% Interpolate results at fixed sampling rate
            tSample = 1/opts.frameRate;
            tQuery = simRes.tout(1):tSample:simRes.tout(end);
            gQuery = interpolateSimResultsTime(obj.system, simRes, tQuery);


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
            [xlimits, ylimits, zlimits] = getAxisLimits(obj, gQuery, margins);
            xlim(xlimits);
            ylim(ylimits);
            zlim(zlimits);

            %% Prepare video capture
            % for better performance

            if opts.saveMovie
                % Get frame dimensions
                obj = obj.updateConfiguration(gQuery(:,:,:,1));
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

                    obj = obj.updateConfiguration(gQuery(:,:,:,iStep));

                    if ~opts.saveMovie
                        % Drawnow is only necessary if we don't save the
                        % animation to a movie since getframe triggers
                        % drawnow
                        drawnow;
                        drawnow;
                    end
                    if isvalid(fig) && opts.saveMovie
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
            if opts.saveMovie
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
                    v = VideoWriter(opts.fileName, 'MPEG-4');
                    v.Quality = 100;
                    v.FrameRate = opts.frameRate;
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
        function [xlimits, ylimits, zlimits] = getAxisLimits(obj, g, margins)
            %% Compute axis limits for simulation results
            arguments
                obj     (1,1)

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
            if obj.system.isCantilever
                x0 = obj.system.g0(1:3,4);
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