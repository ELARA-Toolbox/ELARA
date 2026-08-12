function figHandle = solverStatsODE(simRes, opts)
    %% Plot statistics for an ODE solver
    arguments
        simRes          (1,1) elara.SimulationResults
        opts.nameString (1,1) string = ""
    end
    if ~isempty(simRes.tout)

        if opts.nameString == ""
            nameString = "";
        else
            nameString = strcat(opts.nameString, ": ");
        end

        % Get time step
        h = diff(simRes.tout);

        figHandle = figure( ...
            'Name', strcat(nameString, "Solver Stats ODE"), 'NumberTitle','off');

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
