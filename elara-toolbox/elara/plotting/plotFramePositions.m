function fh = plotFramePositions(obj, opts)
    %% Plot Frame Positions
    % Plot positions of:
    %  * all rigid links (reference/COM frame)
    %  * first and last node of the flexible links
    arguments
        obj          (1,1) elara.Simulation
        opts.nameStr (1,1) string = ""
    end

    fh = figure( ...
        'Name', strcat(opts.nameStr, "Frame Positions"), ...
        'NumberTitle','off');

    tl = tiledlayout(fh, "flow");

    for iLink = 1:obj.system.nLinks
        if obj.links(iLink).isRigid
            iPlotFrames = obj.system.linkFrameIndices(1,iLink);
        else
            iPlotFrames = obj.system.linkFrameIndices(:,iLink);
        end
        for iPlot = 1:numel(iPlotFrames)
            iFrame = iPlotFrames(iPlot);
            if obj.links(iLink).isRigid
                titleString = sprintf('Link %d (Rigid)', iLink );
            else
                iNode = iFrame - obj.system.linkFrameIndices(1,iLink);
                titleString = sprintf("Link %d (Flex.), Node %d", iLink, iNode);
            end

            %%% Plot
            t = nexttile(tl);
            plot(t, ...
                obj.results.tout, reshape( obj.results.g(1:3, 4, iFrame, :), 3, []) ...
                );

            grid on
            xlim([obj.results.tout(1), obj.results.tout(end)]);

            title( titleString, 'Interpreter', 'latex')
            legend('$x$', '$y$', '$z$', 'interpreter', 'latex');
            xlabel('time $t$ / s', 'interpreter', 'latex')
            ylabel('pos. / m', 'interpreter', 'latex')
            grid on
            box on
            %colororder(ax4(1, iTile), plotColors3);
        end
    end
end