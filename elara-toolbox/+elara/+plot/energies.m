function energies(simRes, opts)
    arguments
        simRes          (1,1) elara.SimulationResults
        opts.nameString (1,1) string = ""
    end
    fh = figure( ...
        'Name', strcat(opts.nameStr, 'Energies'), 'NumberTitle','off');

    ax = axes(fh);

    plot(ax, simRes.tout, simRes.totalEnergy, 'LineWidth', 2);
    hold on;
    plot(ax, simRes.tout, simRes.kineticEnergy);
    plot(ax, simRes.tout, simRes.potentialEnergy);
    plot(ax, simRes.tout, simRes.strainEnergy);

    title(ax, 'Energies', 'interpreter', 'latex')
    grid on
    xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')
    ylabel(ax, 'Energy / J', 'interpreter', 'latex')

    legend(ax, 'Total $H$', 'Kinetic $T$', 'Potential $U$', 'Strain $V$', ...
        'interpreter', 'latex');
end
