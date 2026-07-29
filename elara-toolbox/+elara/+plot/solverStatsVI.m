function figHandle = solverStatsVI(simRes, opts)
    %% Plot statistics about the implicit solver
    arguments
        simRes          (1,1) elara.SimulationResults
        opts.nameStr    (1,1) string = ""
    end
    if ~isempty(simRes.solverIterations)

        figHandle = figure( ...
            'Name', strcat(opts.nameStr, "Solver Stats VI"), 'NumberTitle','off');

        t = tiledlayout(figHandle, 2,1);

        % Residual Error
        ax = nexttile(t);
        plot(ax, simRes.tout, simRes.solverResidual');
        title(ax, 'Residual Error', 'interpreter', 'latex')
        xlim(ax, [0, simRes.tout(end)])
        grid on
        xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')

        % Nr. of iterations and error flag of the implicit solver
        ax = nexttile(t);
        yyaxis left
        plot(ax, simRes.tout, simRes.solverIterations', '-o');
        ylabel('Nr. of iterations', 'Interpreter','latex');

        yyaxis right
        plot(ax, simRes.tout, simRes.solverExitFlag', '-o', 'MarkerSize', 3)
        ylabel('Error Flag', 'Interpreter','latex');

        title(ax, 'Nr. of Iteration and Error Flag', 'interpreter', 'latex')
        xlim(ax, [0, simRes.tout(end)])
        grid on

        xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')
    else
        figHandle = gobjects(0);
    end
end
