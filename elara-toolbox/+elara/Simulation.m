classdef Simulation
    % Class that contains everything to perform a complete multibody
    % simulation.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Simulation name (used, e.g., in plots)
        Name            (1,1) string

        %% Used integrator
        % The integrator object contains all methods and settings required
        % to perform the integration
        integrator      (1,1) MBSimIntegrator = MBSimIntegratorVarIntBroyden;

        %% System specification

        % Link definitions
        links   (:,1) elara.internal.Link

        % ELARA system definition
        system   (1,1) elara.SystemNum

        %% Simulation Parameters
        parameters (1,1) elara.SimulationParameters

        %% Simulation Results
        results  (1,1) MBSimResults
    end

    methods
        %% Constructor
        function obj = Simulation(links, options)
            % Create instance of this class
            % If links are given (optionally), assemble the MB system from them
            arguments
                links               (:,1) elara.internal.Link = elara.RigidLink.empty;
                options.displayInfo (1,1) logical           = true;
                options.Name        (1,1) string            = "";
            end

            obj.Name = options.Name;

            if ~isempty(links)
                obj.links = links;
                obj.system = elara.SystemNum(links);

                % Plot system information
                if options.displayInfo
                    printLinkProperties(links);
                    printFrameProperties(obj.system);
                    plotSystemGraphs(obj.system);
                    printInputProperties(obj.system);
                end
            end
        end

        %% Run Simulation
        function obj = simulateSystem(obj)
            arguments
                obj (1,1) elara.Simulation
            end
            obj.results = obj.integrator.simulateSystem(obj);
        end

        %% Visualization

        function [fig, vis] = visualizeSystemRefConf(obj, options)
            % Visualize the system in its reference configuration
            arguments
                obj             (1,1) elara.Simulation

                % Figure Name
                options.figureName    (1,1) string = "Visualization Reference Config";

                % Create new figure?
                options.createFigure  (1,1) logical = true;

                % Show inertial frame of the system?
                options.ShowInertialFrame (1,1) logical = true;
            end
            q = zeros(obj.system.nDoF, 1);
            [fig, vis] = visualizeSystemConfig(obj, q, ...
                "createFigure", options.createFigure, ...
                "figureName", options.figureName, ...
                "ShowInertialFrame", options.ShowInertialFrame);
            title("Reference Configuration")
        end

        function [fig, vis] = visualizeSystemConfig(obj, q, options)
            % Visualize the system in given configuration
            arguments
                obj             (1,1) elara.Simulation
                % Vector of system coordinates
                q               (:,1) double
                % Figure Name
                options.figureName    (1,1) string = "3D Vis.";

                % Create new figure?
                options.createFigure  (1,1) logical = true;

                % Show inertial frame of the system?
                options.ShowInertialFrame (1,1) logical = true;
            end
            g = obj.system.computeFwdKin(q);
            if options.createFigure
                if obj.Name == ""
                    figureName = options.figureName;
                else
                    figureName = strcat(obj.Name, ": ", options.figureName);
                end
                fig = init3Dplot("Name", figureName, "NumberTitle", "off");
            else
                fig = gcf;
            end
            vis = MBSysVisualization(obj.system, obj.links, g, ...
                "ShowInertialFrame", options.ShowInertialFrame);
        end

        function MBVisAnim = animateSimResults(obj, opts)
            %% Animate simulation results
            arguments
                obj

                % Figure Name
                opts.figureName     (1,1) string = "Animation";

                % Figure to plot in
                opts.figure         (:,1) matlab.ui.Figure = init3Dplot;

                % Animation frame rate
                opts.frameRate      (1,1) double = 30;

                % Save animation to file?
                opts.saveMovie      (1,1) logical = false;

                % File name for the saved movie (full path)
                opts.fileName       (1,1) string = "animation";
            end
            fig = opts.figure;
            fig.NumberTitle = "off";
            if isempty(fig.Name) || fig.Name == ""
                if obj.Name == ""
                    fig.Name = opts.figureName;
                else
                    fig.Name = strcat(obj.Name, ": ", opts.figureName);
                end
            end
            % Change docked figure windows to separate window?
            %fig.WindowStyle = "normal";

            MBVisAnim = MBSysVisualization(obj.system, obj.links);

            MBVisAnim.animateSystem(obj.results, fig, ...
                "saveMovie", opts.saveMovie, ...
                "fileName", opts.fileName, ...
                "frameRate", opts.frameRate);
        end

        function [fig, vis] = drawSnapshots(obj, opts)
            arguments
                obj             (1,1)

                % Figure to plot in
                opts.figure     (:,1) matlab.ui.Figure = init3Dplot;

                % Figure Name (for newly created figure)
                opts.figureName (1,1) string = "Snapshots";

                opts.nSnapShots (1,1) double = 25;

                % Include colorbar?
                opts.includeColorbar (1,1) logical = true;

                % Function handle for the color map used in the snapshots
                opts.snapShotColormap (1,1) function_handle = @winter;

                % Save animation to file?
                %opts.saveMovie  (1,1) logical = false;

                % File name for the saved movie (full path)
                %opts.fileName   (1,1) string = "animation";
            end

            fig = opts.figure;
            if isempty(fig.Name)
                if obj.Name == ""
                    fig.Name = opts.figureName;
                else
                    fig.Name = strcat(obj.Name, ": ", opts.figureName);
                end
            end
            axSnapShots = gca;

            %% Interpolate results at fixed sampling rate
            tQuery = linspace(obj.results.tout(1), obj.results.tout(end), opts.nSnapShots);
            gQuery = interpolateSimResultsTime(obj.system, obj.results, tQuery);

            %% Prepare axis limits
            % Extra margin
            margins = [-1, -1, -1; 1, 1, 1]*0.1;

            % Set limits
            [xlimits, ylimits, zlimits] = getAxisLimits(obj, gQuery, margins);
            xlim(xlimits);
            ylim(ylimits);
            zlim(zlimits);

            % Get colors
            snapShotColors = opts.snapShotColormap(size(gQuery,4)+1);

            %% Draw Snapshots
            for iStep = 1:length(tQuery)
                vis = MBSysVisualization( ...
                    obj.system, obj.links, gQuery(:,:,:,iStep), ...
                    "ShowInertialFrame", false, ...
                    "ShowJoints", false, ...
                    "ShowLinkFrames", false, ...
                    "ShowTendons", false ...
                    );

                for iLink = 1:length(vis.linkVis)
                    % Set colors of all links
                    vis.linkVis(iLink).Color = snapShotColors(iStep, :);

                    % Disable labels of TCP frame
                    vis.linkVis(iLink).cSysTCP.DrawLabels = false;
                    vis.linkVis(iLink).cSysTCP.Visible = false;
                end
            end

            %% Set figure properties
            title("Configuration Snapshots", "Interpreter", "latex");
            xlabel('$x$ in m','Interpreter','latex');
            ylabel('$y$ in m','Interpreter','latex');
            zlabel('$z$ in m','Interpreter','latex');

            view([37.5, 30]);

            % Add colorbar, if required
            if opts.includeColorbar
                axSnapShots.Colormap = opts.snapShotColormap();
                ch = colorbar;
                ch.Label.Interpreter = 'latex';
                ch.TickLabelInterpreter = 'latex';
                ch.FontSize = 10;
                ch.Position(3) = 0.75*ch.Position(3);
                ch.Position(4) = 0.95*ch.Position(4);
                clim(axSnapShots, [obj.results.tout(1), obj.results.tout(end)]);
                ylabel(ch, 'time $t$ in s', 'Interpreter', 'latex','FontSize', 10);
            end
        end

        %% Post-Processing
        function obj = computeEnergies(obj, opts)
            %% Compute energy evolution for a finished simulation
            arguments
                obj
                opts.useFD (1,1) logical = true;
            end
            disp('   Computing Energy Evolution...');
            isVarInt = obj.integrator.type ==  "varint";

            % Check if compiled mex files are available
            if exist("computeFirstOrderSystemRHS_mex", "file")
                obj.results.energies = computeSimResEnergies_mex( ...
                    obj.system, obj.parameters, obj.results, isVarInt, opts.useFD);
            else
                obj.results.energies = computeSimResEnergies( ...
                    obj.system, obj.parameters, obj.results, isVarInt, opts.useFD);
            end
        end


        %% Plotting
        function fhs = plotAll(obj)
            %% Generate all available plots for the simulation
            if obj.Name == ""
                nameStr = "";
            else
                nameStr = strcat(obj.Name, ": ");
            end
            % Various plots
            fhs(1) = plotJointAngles(obj.system, obj.results, "nameStr", nameStr);
            fhs(2) = plotFramePositions(obj, "nameStr", nameStr);
            fhs_vel = plotFrameVelocities(obj, "nameStr", nameStr);
            fhs_beam = plotBeamData(obj, "nameStr", nameStr);

            % Solver stats: Depend on the chosen solver
            switch obj.integrator.type
                case "varint"
                    fh_stats = plotSolverStatsVI(obj.results, "nameStr", nameStr);
                case "ode"
                    fh_stats = plotSolverStatsODE(obj.results, "nameStr", nameStr);
                otherwise
                    error("Solver not implemented.");
            end
            fhs = [fhs, fhs_vel, fhs_beam, fh_stats];
        end
    end
    methods(Hidden)
        %% Helper methods
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