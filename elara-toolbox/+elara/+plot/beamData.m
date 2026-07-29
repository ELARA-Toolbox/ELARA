function figHandles = beamData(sim, opts)
    %% Plot Beam node / segment data
    arguments
        sim             (1,1) elara.Simulation
        opts.nameString (1,1) string = ""
    end

    figHandles = gobjects(0);

    for iLink = 1:sim.system.nLinks
        if ~sim.links(iLink).isRigid
            % Prepare struct with beam data to use (legacy) beam
            % plotting function
            frameIndices = sim.system.linkFrameIndices(1,iLink):sim.system.linkFrameIndices(2,iLink);
            simData.tout = sim.results.tout;
            simData.R = sim.results.g(1:3,1:3,frameIndices,:);
            simData.x = squeeze(sim.results.g(1:3,4,frameIndices,:));
            simData.eta = sim.results.eta(:,frameIndices,:);

            simData.xi = zeros(6,sim.links(iLink).nSegments, length(sim.results.tout));
            for iStep = 1:length(sim.results.tout)
                simData.xi(:,:,iStep) = sim.system.getLinkDeformations(sim.results.q(:,iStep), iLink);
            end

            if iLink == 1 && sim.system.isCantilever
                simData.R = cat(3, nan(3,3,1,length(sim.results.tout)), simData.R);
                simData.x = cat(2, nan(3,1,length(sim.results.tout)), simData.x);
                simData.eta = cat(2, zeros(6,1,length(sim.results.tout)), simData.eta);
            end
            if opts.nameString == ""
                nameStr = sprintf("Link %d: ", iLink);
            else
                nameStr = sprintf("%sLink %d: ", opts.nameString, iLink);
            end
            fhs = plotBeamNodeData(simData, sim.links(iLink).xiRef, "name", nameStr);

            figHandles = [figHandles, fhs];
        end
    end
end

function figHandles = plotBeamNodeData(simData, xiRef, opts)
    %% Plot various data from beam nodes
    arguments
        % Can be both object or simple struct
        simData
        xiRef
        opts.name (1,1) string = ""
    end

    nameStr = opts.name;

    % Get nr. of nodes and segments
    nNodes = size(simData.R, 3);
    nSegments   = size(simData.xi,2);

    % Colors for plots with 3-dimensional data
    plotColors3 = lines(3);


    %% Get nodes / segments to plot
    % For beams with a high node number, don't plot the variables for
    % individual nodes/segments for all nodes, but only a maximum of n
    % segments / nodes. If there are more segments/nodes, skip some.

    nPlots = 5;

    if nNodes > nPlots
        plotSegments = round(linspace(1, nSegments, nPlots));
        plotNodes    = round(linspace(1, nNodes, nPlots));

        % Plot 2nd instead of first node since 1st node is always zero
        %plotNodes(1) = 2;
    else
        plotSegments = 1:nSegments;
        plotNodes = 1:nNodes;
    end


    %% Configuration (individual)

    fh = figure( ...
        'Name', strcat(nameStr, 'Configuration Indiv'), 'NumberTitle','off');

    t = tiledlayout(fh, 2, length(plotNodes), ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    title(t, 'Node Configuration $g_a$ (absolute)', ...
        'interpreter', 'latex');

    ax2 = gobjects(2, length(plotNodes));

    for iTile = 1:length(plotNodes)

        iN = plotNodes(iTile);

        %%% Plot rotation matrices R
        ax2(1, iTile) = nexttile(t, iTile);

        plot(ax2(1, iTile), ...
            simData.tout, reshape(simData.R(:,:,iN,:),[9, length(simData.tout)]) ...
            );

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        ylim([-1,1]);
        title( sprintf('Rot. Matrix $R_{%d}$', iN ), ...
            'Interpreter', 'latex')


        %%% Plot positions x
        ax2(2, iTile) = nexttile(t, iTile + length(plotNodes));

        plot(ax2(2, iTile), ...
            simData.tout, reshape( simData.x(:, iN, :), 3, []) ...
            );

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        title( sprintf('Position $x_{%d}$', iN ), ...
            'Interpreter', 'latex')
        legend('$x$', '$y$', '$z$', 'interpreter', 'latex');

        xlabel('time $t$ / s', 'interpreter', 'latex')
        colororder(ax2(2, iTile), plotColors3);
    end

    figHandles = fh;


    %% Velocities (individual)

    fh = figure( ...
        'Name', strcat(nameStr, 'Velocities Indiv'), 'NumberTitle','off');

    t = tiledlayout(fh, 2, length(plotNodes), ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    title(t, 'Node Velocities $\eta_a$ (body-fixed frame)', ...
        'interpreter', 'latex');

    ax4 = gobjects(2, length(plotNodes));

    for iTile = 1:length(plotNodes)

        iN = plotNodes(iTile);

        %%% Plot rotational parts
        ax4(1, iTile) = nexttile(t, iTile);

        plot(ax4(1, iTile), ...
            simData.tout, reshape( simData.eta(1:3, iN, :), 3, []) ...
            );

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        title( sprintf('Rot., Node %d', iN ), 'Interpreter', 'latex')
        legend('$x$', '$y$', '$z$', 'interpreter', 'latex');
        colororder(ax4(1, iTile), plotColors3);


        %%% Plot translational parts
        ax4(2, iTile) = nexttile(t, iTile + length(plotNodes));

        plot(ax4(2, iTile), ...
            simData.tout, reshape( simData.eta(4:6, iN, :), 3, []) ...
            );

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        title( sprintf('Transl., Node %d', iN ), 'Interpreter', 'latex')
        legend('$x$', '$y$', '$z$', 'interpreter', 'latex');

        xlabel('time $t$ / s', 'interpreter', 'latex')
        colororder(ax4(2, iTile), plotColors3);
    end

    figHandles = [figHandles, fh];


    %% Discrete Deformations xi (individual)
    if 0
        fh = figure( ...
            'Name', strcat(nameStr, 'Discrete Deformations'), 'NumberTitle','off');

        t = tiledlayout(fh, 2, length(plotSegments), ...
            'TileSpacing', 'compact', 'Padding', 'compact');

        title(t, 'Discrete Displacements $\xi_a$', 'interpreter', 'latex');

        ax5 = gobjects(2, length(plotSegments));

        for iTile = 1:length(plotSegments)

            iSeg = plotSegments(iTile);

            %%% Plot rotational parts
            ax5(1, iTile) = nexttile(t, iTile);

            % Actual deformation
            plot(ax5(1, iTile), ...
                simData.tout, reshape(simData.xi(1:3,iSeg,:), 3, length(simData.tout)) ...
                );
            hold on

            % Reference deformation
            plot( simData.tout(end) *[0;1] , [xiRef(1:3, iSeg), xiRef(1:3, iSeg)], '--' );

            grid on
            xlim([simData.tout(1), simData.tout(end)]);
            title( sprintf('Rot., Seg. %d', iSeg ), 'Interpreter', 'latex')
            legend(...
                '$\alpha$', '$\beta$', '$\gamma$', ...
                '$\bar{\alpha}$', '$\bar{\beta}$', '$\bar{\gamma}$', ...
                'interpreter', 'latex');
            colororder(ax5(iTile), plotColors3);


            %%% Plot translational parts
            ax5(2, iTile) = nexttile(t, iTile + length(plotSegments));

            % Actual deformation
            plot(ax5(2, iTile), ...
                simData.tout, reshape(simData.xi(4:end,iSeg,:), [3, length(simData.tout)]) ...
                );
            hold on

            % Reference deformation
            plot(ax5(2, iTile), ...
                simData.tout(end) *[0;1] , [xiRef(4:6, iSeg), xiRef(4:6, iSeg)], '--' ...
                );

            grid on
            xlim([simData.tout(1), simData.tout(end)]);
            title( sprintf('Transl., Seg. %d', iSeg ), 'Interpreter', 'latex')
            legend( ...
                '$x$', '$y$', '$z$', '$\bar{x}$', '$\bar{y}$', '$\bar{z}$',...
                'interpreter', 'latex');

            xlabel('time $t$ / s', 'interpreter', 'latex')
            colororder(ax5(iTile), plotColors3);
        end

        linkaxes(ax5(1,:));
        linkaxes(ax5(2,:));
    end

    figHandles = [figHandles, fh];


    %% Plot Strains (xi-xiRef) (individual)

    fh = figure( ...
        'Name', strcat(nameStr, 'Discrete Strains'), 'NumberTitle','off');

    t = tiledlayout(fh, 2, length(plotSegments), ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    title(t, 'Discrete Strains $(\xi_a - \bar{\xi}_a)$', ...
        'interpreter', 'latex');

    ax6 = gobjects(2, length(plotSegments));

    %l = params.L / nSegments;

    for iTile = 1:length(plotSegments)

        iSeg = plotSegments(iTile);

        % Compute Strains
        strains = ...%1 / l * params.Cgen * ...
            ( reshape(simData.xi(:,iSeg,:), [6, length(simData.tout)]) - repmat(xiRef(:, iSeg), [1, length(simData.tout)]) ...
            );


        % Plot rotational parts
        ax6(1, iTile) = nexttile(t, iTile);

        plot(ax6(1, iTile), simData.tout, strains(1:3,:));

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        title( sprintf('Rot., Seg. %d', iSeg ), 'Interpreter', 'latex')
        legend(...
            '$\alpha$', '$\beta$', '$\gamma$', 'interpreter', 'latex');
        colororder(ax6(1, iTile), plotColors3);


        % Plot translational parts
        ax6(2, iTile) = nexttile(t, iTile + length(plotSegments));

        plot(ax6(2, iTile), simData.tout, strains(4:6,:));

        grid on
        xlim([simData.tout(1), simData.tout(end)]);
        title( sprintf('Transl., Seg. %d', iSeg ), 'Interpreter', 'latex')
        legend( ...
            '$x$', '$y$', '$z$', 'interpreter', 'latex');

        xlabel('time $t$ / s', 'interpreter', 'latex')
        colororder(ax6(2, iTile), plotColors3);
    end

    figHandles = [figHandles, fh];


    %% Synchronize axes of the subplots
    % Do this at the end of script for all figures simultaneously to save
    % time (since linkaxes calls drawnow, which takes some time)

    linkaxes(ax2(1,:));
    linkaxes(ax2(2,:));

    linkaxes(ax4(1,:));
    linkaxes(ax4(2,:));

    linkaxes(ax6(1,:));
    linkaxes(ax6(2,:));

end
