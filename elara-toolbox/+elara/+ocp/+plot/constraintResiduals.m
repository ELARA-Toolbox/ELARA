function fh = constraintResiduals(OCP, x, u_z, opts)
    %% Plot OCP constraint residuals for a given trajectory
    arguments
        OCP         (1,1) elara.ocp.Problem

        % Configuration or state trajectory
        % Variational discretization: configurations q, (nDoF, nSteps+1)
        % ODE discretization: states x = [q; qDot], (2*nDoF, nSteps+1)
        x       (:,:) double

        % Control decision variables
        % Direct parameterization: time-node values, (nInputs, nSteps+1)
        % Spline parameterization: B-spline control points (nInputs, nSplinePoints)
        u_z     (:,:) double

        opts.figureName (1,1) string = "Constraint Residuals";
    end
    constrDef = OCP.constrDef;

    %% Compute residuals

    if OCP.useSplineInputs
        decisionVector = elara.internal.ocp.packSplineDecisionVariables(x, u_z);
    else
        decisionVector = elara.internal.ocp.packNodeDecisionVariables(x, u_z);
    end
    res_c = full(constrDef.Fun_c(decisionVector, OCP.x_TCP_F));
    res_cDyn = full(constrDef.Fun_cDyn(decisionVector));
    res_cWS_int = full(constrDef.Fun_cWS_int(decisionVector, OCP.x_TCP_F));
    res_cWS_ext = full(constrDef.Fun_cWS_ext(decisionVector, OCP.x_TCP_F));

    % Compute violations of lower and upper bounds
    cV_ub = nan(size(res_c));
    cV_ub(res_c>constrDef.ub_c) = abs(res_c(res_c>constrDef.ub_c) - constrDef.ub_c(res_c>constrDef.ub_c));
    cV_lb = nan(size(res_c));
    cV_lb(res_c<constrDef.lb_c) = abs(res_c(res_c<constrDef.lb_c) - constrDef.lb_c(res_c<constrDef.lb_c));

    %% Plot

    fh = figure("NumberTitle", "off");
    if OCP.Name == ""
        fh.Name = opts.figureName;
    else
        fh.Name = strcat(OCP.Name, ": ", opts.figureName);
    end

    tiledlayout("vertical");
    nexttile;
    semilogy(abs(res_c), "DisplayName", "$c(x)$");
    hold on;
    semilogy(cV_ub, '.-', "DisplayName", "upper-bound violation");
    semilogy(cV_lb, '.-', "DisplayName", "lower-bound violation");
    title("Constraint function values")
    xlabel("function element", "Interpreter", "latex");
    ylabel('$|c(x)|$', "Interpreter", "latex");
    legend("Interpreter", "latex", "location", "best");
    grid on;
    xlim([0, numel(res_c)]);

    nexttile;
    semilogy(vecnorm(res_cDyn), "DisplayName", "dynamics-constraint norm");
    title("System-dynamics constraint residual norms")
    xlabel("time step $k$", "Interpreter", "latex");
    ylabel("constraint residuals norm $||c(x)||$", "Interpreter", "latex");
    grid on;
    xlim([0, OCP.nSteps+1]);


    if ~isempty(res_cWS_int) || ~isempty(res_cWS_ext)
        nexttile;
        if ~isempty(res_cWS_int)
            plot((res_cWS_int).', "DisplayName", "interior ($c(x) > 0$)");
        end
        if ~isempty(res_cWS_ext)
            plot(res_cWS_ext.', "DisplayName", "exterior ($c(x) < 0$)");
        end
        grid on
        xlabel("time step $k$", "Interpreter", "latex");
        ylabel("constraint residuals $c(x)$", "Interpreter", "latex");
        legend("Interpreter", "latex")
        title("Workspace constraints")
        xlim([0, OCP.nSteps+1]);
    end
    drawnow;
end
