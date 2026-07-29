function figHandles = frameVelocities(sim, opts)
    %% Plot Frame Velocities
    % Plot velocities of:
    %  * all rigid links (reference/COM frame)
    %  * first and last node of the flexible links
    arguments
        sim             (1,1) elara.Simulation
        opts.nameString (1,1) string = ""
    end

    fh_angVel = figure( ...
        'Name', strcat(opts.nameString, "Frame Angular Velocities"), ...
        'NumberTitle','off');
    fh_trVel = figure( ...
        'Name', strcat(opts.nameString, "Frame Transl. Velocities"), ...
        'NumberTitle','off');

    tl_angVel = tiledlayout(fh_angVel, "flow");
    tl_trVel  = tiledlayout(fh_trVel, "flow");

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

            %%% Plot angular velocity
            t = nexttile(tl_angVel);
            plot(t, ...
                sim.results.tout, reshape( sim.results.eta(1:3, iFrame, :), 3, []) ...
                );

            grid on
            xlim([sim.results.tout(1), sim.results.tout(end)]);

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
                sim.results.tout, reshape( sim.results.eta(4:6, iFrame, :), 3, []) ...
                );

            grid on
            xlim([sim.results.tout(1), sim.results.tout(end)]);

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
