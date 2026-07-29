function fh = framePositions(sim, opts)
    %% Plot Frame Positions
    % Plot positions of:
    %  * all rigid links (reference/COM frame)
    %  * first and last node of the flexible links
    arguments
        sim             (1,1) elara.Simulation
        opts.nameString (1,1) string = ""
    end

    % Generate figure name string
    if opts.nameString == ""
        nameString = "";
    else
        nameString = strcat(opts.nameString, ": ");
    end

    fh = figure( ...
        'Name', strcat(nameString, "Frame Positions"), ...
        'NumberTitle','off');

    tl = tiledlayout(fh, "flow");

    for iLink = 1:sim.system.nLinks
        if sim.links(iLink).isRigid
            iPlotFrames = sim.system.linkFrameIndices(1,iLink);
        else
            iPlotFrames = sim.system.linkFrameIndices(:,iLink);
        end
        for iPlot = 1:numel(iPlotFrames)
            iFrame = iPlotFrames(iPlot);
            if sim.links(iLink).isRigid
                titleString = sprintf('Link %d (Rigid)', iLink );
            else
                iNode = iFrame - sim.system.linkFrameIndices(1,iLink);
                titleString = sprintf("Link %d (Flex.), Node %d", iLink, iNode);
            end

            %%% Plot
            t = nexttile(tl);
            plot(t, ...
                sim.results.tout, reshape( sim.results.g(1:3, 4, iFrame, :), 3, []) ...
                );

            grid on
            xlim([sim.results.tout(1), sim.results.tout(end)]);

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
