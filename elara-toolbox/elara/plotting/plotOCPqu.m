function fhs = plotOCPqu(OCP, q, u, opts)
    %% Plot coordinates and inputs of a solved OCP
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        OCP     (1,1) elara.ocp.Problem

        % Coordinates with dimensions (nDoF, nSteps+1)
        q    (:,:) double

        % Inputs with dimensions (nDoF, nSteps+1)
        u    (:,:) double

        % Coordinate velocities
        % If left empty, discrete velocities are automatically computed and
        % used
        opts.q_dot (:,:) double = []

        opts.figureName (1,1) string = "Solution";

        % Options to also plot the (discrete) derivatives in separate
        % figures
        opts.plotDerivatives (1,1) logical = false;

        % Finite difference order for the derivative plots
        opts.FDOrder (1,1) double {mustBeMember(opts.FDOrder, [2,4])} = 4;
    end

    % Compute discrete velocities
    q_dot_sol_d = diff(q, 1, 2) / OCP.h;

    if OCP.Name == ""
        figName = opts.figureName;
    else
        figName = strcat(OCP.Name, ": ", opts.figureName);
    end

    %% Standard Plot

    fhs = figure("NumberTitle", "off");
    fhs.Name = figName;
    tiledlayout("vertical");

    % Coordinates
    nexttile;
    plot(OCP.tout, q)
    grid on
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    ylabel('$q^k$', 'Interpreter', 'latex');

    title("coordinates", "interpreter", "latex");
    legend(arrayfun(@(x) sprintf("$q_{%d}$", x), 1:size(q,1)), "Interpreter", "latex");

    % Coordinate Velocities
    ax = nexttile;
    stairs(OCP.tout, [q_dot_sol_d,q_dot_sol_d(:,end)].');
    ylabel('$(q^{k+1}-q^k)/h$', 'Interpreter', 'latex');
    title("(discrete) velocities", "interpreter", "latex");

    if ~isempty(opts.q_dot)
        hold on;
        plot(OCP.tout, opts.q_dot, "--");
    end
    ax.ColorOrder = lines(size(q, 1));
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    grid on

    % Inputs
    ax = nexttile;
    plot(OCP.tout, u);
    ylabel('$u^k$', 'Interpreter', 'latex');
    title("inputs", "interpreter", "latex");
    ax.ColorOrder = lines(size(u, 1));
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    grid on
    legend(arrayfun(@(x) sprintf("input $u_{%d}$", x), 1:size(u,1)), 'Interpreter', 'latex');

    % Add markers to indicate B-spline control points
    if OCP.useSplineInputs
        % Get Greville abscissae points as time values for the control
        % points (average time value of the time span influenced by a
        % control point)
        [~,~,~,tCP] = OCP.getInputSplineBasisMatrix;

        % Get data value at Greville points
        uCP = interp1(OCP.tout, u.', tCP.');

        hold on;
        plot(tCP, uCP, "o", "HandleVisibility", "off");
    end


    %% Plot derivatives

    if opts.plotDerivatives
        switch opts.FDOrder
            case 2
                [qd, qdd] = diff2ndOrder(q, OCP.h);
                [ud, udd] = diff2ndOrder(u, OCP.h);
            case 4
                [qd, qdd] = diff4thOrder(q, OCP.h);
                [ud, udd] = diff4thOrder(u, OCP.h);
        end
        fhs(2) = figure("NumberTitle", "off");
        fhs(2).Name = figName + " Derivatives";

        % See https://www.mathworks.com/matlabcentral/answers/2044242-how-to-create-tiledlayout-grid-in-vertical-order
        % for "tileindexing"
        tiledlayout(3, 2, 'TileIndexing', 'columnmajor');
        plotDerivative(OCP.tout, u, ud, udd, "inputs", "u");
        plotDerivative(OCP.tout, q, qd, qdd, "coordinates", "q");
    end
end


function ax = plotDerivative(tout, x, xd, xdd, varName, varSymbol)
    %% Plot derivatives of a variable x

    arguments
        tout        (:,1) double
        x           (:,:) double
        xd          (:,:) double
        xdd         (:,:) double
        varName     (1,1) string
        varSymbol   (1,1) string
    end

    % Variable
    ax(1) = nexttile;
    plot(tout, x)
    grid on
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    ylabel(sprintf("$%s$", varSymbol), 'Interpreter', 'latex');
    title(varName, "interpreter", "latex");
    legend(arrayfun(@(x) sprintf("$%s_{%d}$", varSymbol, x), 1:size(x,1)), "Interpreter", "latex");

    % First derivative
    ax(2) = nexttile;
    plot(tout, xd)
    grid on
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    ylabel(sprintf("$\\dot{%s}$", varSymbol), 'Interpreter', 'latex');
    title(sprintf("First derivative %s", varName), "interpreter", "latex");

    % Second derivative
    ax(3) = nexttile;
    plot(tout, xdd)
    grid on
    xlabel('time $t$ in s', 'Interpreter', 'latex');
    ylabel(sprintf("$\\ddot{%s}$", varSymbol), 'Interpreter', 'latex');
    title(sprintf("Second derivative %s", varName), "interpreter", "latex");

end