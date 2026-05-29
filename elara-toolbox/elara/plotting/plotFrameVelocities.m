function figHandles = plotFrameVelocities(obj, opts)
    %% Plot Frame Velocities
    % Plot velocities of:
    %  * all rigid links (reference/COM frame)
    %  * first and last node of the flexible links
    arguments
        obj          (1,1) MBSimulation
        opts.nameStr (1,1) string = ""
    end

    fh_angVel = figure( ...
        'Name', strcat(opts.nameStr, "Frame Angular Velocities"), ...
        'NumberTitle','off');
    fh_trVel = figure( ...
        'Name', strcat(opts.nameStr, "Frame Transl. Velocities"), ...
        'NumberTitle','off');

    tl_angVel = tiledlayout(fh_angVel, "flow");
    tl_trVel  = tiledlayout(fh_trVel, "flow");

    for iLink = 1:obj.MBSys.nLinks
        if obj.links(iLink).isRigid
            iPlotFrames = obj.MBSys.linkFrameIndices(1,iLink);
        else
            iPlotFrames = obj.MBSys.linkFrameIndices(:,iLink);
        end
        for iPlot = 1:numel(iPlotFrames)
            iFrame = iPlotFrames(iPlot);
            if obj.links(iLink).isRigid
                titleString = sprintf('Link %d (Rigid)', iLink );
            else
                iNode = iFrame - obj.MBSys.linkFrameIndices(1,iLink);
                titleString = sprintf("Link %d (Flex.), Node %d", iLink, iNode);
            end

            %%% Plot angular velocity
            t = nexttile(tl_angVel);
            plot(t, ...
                obj.simRes.tout, reshape( obj.simRes.eta(1:3, iFrame, :), 3, []) ...
                );

            grid on
            xlim([obj.simRes.tout(1), obj.simRes.tout(end)]);

            title( titleString, 'Interpreter', 'latex')
            legend('$x$', '$y$', '$z$', 'interpreter', 'latex');
            xlabel('time $t$ / s', 'interpreter', 'latex')
            ylabel('$\omega_i$ / rad / s', 'interpreter', 'latex')
            grid on
            box on
            %colororder(ax4(1, iTile), plotColors3);

            %%% Plot translational velocity
            t = nexttile(tl_trVel);
            plot(t, ...
                obj.simRes.tout, reshape( obj.simRes.eta(4:6, iFrame, :), 3, []) ...
                );

            grid on
            xlim([obj.simRes.tout(1), obj.simRes.tout(end)]);

            title( titleString, 'Interpreter', 'latex')
            legend('$x$', '$y$', '$z$', 'interpreter', 'latex');
            xlabel('time $t$ / s', 'interpreter', 'latex')
            ylabel('$v_i$ / m/s', 'interpreter', 'latex')
            grid on
            box on
        end
    end
    figHandles = [
        fh_angVel, fh_trVel,
        ];
end