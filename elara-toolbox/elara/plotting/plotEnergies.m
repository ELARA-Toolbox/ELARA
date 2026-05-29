function plotEnergies(simRes, opts)
    arguments
        simRes          (1,1) MBSimResults
        opts.nameStr (1,1) string = ""
    end
    fh = figure( ...
        'Name', strcat(opts.nameStr, 'Energies'), 'NumberTitle','off');

    ax = axes(fh);

    plot(ax, simRes.tout, simRes.energies.H, 'LineWidth', 2);
    hold on;
    plot(ax, simRes.tout, simRes.energies.T);
    plot(ax, simRes.tout, simRes.energies.U);
    plot(ax, simRes.tout, simRes.energies.V);

    title(ax, 'Energies', 'interpreter', 'latex')
    grid on
    xlabel(ax, 'time $t$ / s', 'interpreter', 'latex')
    ylabel(ax, 'Energy / J', 'interpreter', 'latex')

    legend(ax, 'Total $H$', 'Kinetic $T$', 'Potential $U$', 'Strain $V$', ...
        'interpreter', 'latex');
end