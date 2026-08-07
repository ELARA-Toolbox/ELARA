function fh = constraintResiduals(OCP, x, u_z, opts)
    %% Plot OCP Constraints residuals for a given trajectory
    arguments
        OCP         (1,1) elara.ocp.Problem

        % State trajectory
        % For varInt discretization: (nDoF, nSteps+1)
        % For ODE discretization:    (2*nDoF, nSteps+1)
        x       (:,:) double

        % Control parameters trajectory
        % Direct parameterization: (nInputs, nSteps+1) (u_sol_z = u_sol)
        % Spline parameterization: (nInputs, nSplinePoints)
        u_z     (:,:) double

        opts.figureName (1,1) string = "Constraint Residuals";
    end
    constrDef = OCP.constrDef;

    %% Compute residuals

    if OCP.useSplineInputs
        xVec = elara.internal.ocp.packSplineDecisionVariables(x, u_z);
    else
        xVec = elara.internal.ocp.packNodeDecisionVariables(x, u_z);
    end
    res_c = full(constrDef.Fun_c(xVec, OCP.x_TCP_F));
    res_cDyn = full(constrDef.Fun_cDyn(xVec));
    res_cWS_int = full(constrDef.Fun_cWS_int(xVec, OCP.x_TCP_F));
    res_cWS_ext = full(constrDef.Fun_cWS_ext(xVec, OCP.x_TCP_F));

    % Compute violations of lower and upper bounds
    cV_ub = nan(size(res_c));
    cV_ub(res_c>constrDef.ub_c) = abs(res_c(res_c>constrDef.ub_c) - constrDef.ub_c(res_c>constrDef.ub_c));
    cV_lb = nan(size(res_c));
    cV_lb(res_c<constrDef.lb_c) = abs(res_c(res_c<constrDef.lb_c) - constrDef.lb_c(res_c<constrDef.lb_c));

    %disp("Norm constraint function c(x):")
    %disp(norm(abs(res_c)));

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
    semilogy(cV_ub, '.-', "DisplayName", "violation ub");
    semilogy(cV_lb, '.-', "DisplayName", "violation lb");
    title("constraint function values")
    xlabel("time step $k$", "Interpreter", "latex");
    ylabel('$|c(x)|$', "Interpreter", "latex");
    legend("Interpreter", "latex", "location", "best");
    grid on;

    nexttile;
    semilogy(vecnorm(res_cDyn), "DisplayName", "sys. dyn. constraints norm");
    title("norm system dynamics constraint residuals")
    xlabel("time step $k$", "Interpreter", "latex");
    ylabel("constraint residuals norm $||c(x)||$", "Interpreter", "latex");
    grid on;

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
    end
    drawnow;
end
