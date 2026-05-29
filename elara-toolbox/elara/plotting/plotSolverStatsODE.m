function figHandle = plotSolverStatsODE(simRes, opts)
    %% Plot statistics about for ODE solver
    arguments
        simRes          (1,1) MBSimResults
        opts.nameStr    (1,1) string = ""
    end
    if ~isempty(simRes.tout)
        % Get time step
        h = diff(simRes.tout);

        figHandle = figure( ...
            'Name', strcat(opts.nameStr, "Solver Stats ODE"), 'NumberTitle','off');

        % Plot
        plot(simRes.tout(1:end-1), h);
        title('ODE Time Step', 'interpreter', 'latex')
        xlim([0, simRes.tout(end)])
        grid on
        xlabel('time $t$ / s', 'interpreter', 'latex')
        ylabel('time step $h$ / s', 'Interpreter','latex');
    else
        figHandle = gobjects(0);        
    end
end
