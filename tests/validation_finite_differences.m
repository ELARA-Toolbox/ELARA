%% Validate finite difference computation
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Example trajectory

h = 1e-3;
tout = 0:h:5;

usePoly = false;
if usePoly
    % Generate example trajectory based on polynomials
    nWayPoints = 6;
    wpts = rand(2,nWayPoints);
    tpts = linspace(0, tout(end), nWayPoints);
    [q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tout);

else
    % Generate example trajectory based on sine function
    % Advantage: "Smoother" curve with nicer numerics
    syms t

    f = [
        3*sin(5*t) + 1/10*t^3  
        -2.5*cos(4*t) - 2/10*t^3
        ];
    f_dt  = diff(f, t, 1);
    f_ddt = diff(f, t, 2);

    fun_f     = matlabFunction(f);
    fun_f_dt  = matlabFunction(f_dt);
    fun_f_ddt = matlabFunction(f_ddt);

    q   = fun_f(tout);
    qd  = fun_f_dt(tout);
    qdd = fun_f_ddt(tout);
end


%% Compute numerical derivatives

FDOrders = [1,2,4];
qd_fd  = cell(size(FDOrders));
qdd_fd = cell(size(FDOrders));

for iO = 1:numel(FDOrders)
    [qd_fd{iO}, qdd_fd{iO}] = diffHigherOrder(q, h, FDOrders(iO));
end


%% Plot Quantities

% Plot plain curves
figure;
tiledlayout("vertical");
nexttile;
plot(tout, q);
grid on
title("Coordinates");
nexttile;
plot(tout, qd);
grid on
title("Velocities");
nexttile;
plot(tout, qdd);
grid on
title("Accelerations");

% Comparison and Errors First Derivative
figure;
tiledlayout("vertical");
nexttile;
plot(tout, qd, "DisplayName", "Exact");
hold on
for iO = 1:numel(FDOrders)
    plot(tout, qd_fd{iO}, '--', "DisplayName", sprintf("Order %d", FDOrders(iO)));
end
grid on
title("Velocities");
legend()
nexttile;
for iO = 1:numel(FDOrders)
    semilogy(tout, abs(qd-qd_fd{iO}), '--', "DisplayName", sprintf("Error Order %d", FDOrders(iO)));
    hold on;
end
title("Absolute Error Velocities");
legend()
grid on;


% Comparison and Errors Second Derivative
figure;
tiledlayout("vertical");
nexttile;
plot(tout, qdd, "DisplayName", "Exact");
hold on
for iO = 1:numel(FDOrders)
    plot(tout, qdd_fd{iO}, '--', "DisplayName", sprintf("Order %d", FDOrders(iO)));
end
grid on
title("Accelerations");
legend()

nexttile;
for iO = 1:numel(FDOrders)
    semilogy(tout, abs(qdd-qdd_fd{iO}), '--', "DisplayName", sprintf("Error Order %d", FDOrders(iO)));
    hold on;
end
title("Absolute Error Accelerations");
legend()
grid on;

drawnow;

%% Quick error order plot

hVec = logspace(-1, -5, 20);

error_qd = zeros(length(hVec), numel(FDOrders));
error_qdd = zeros(length(hVec), numel(FDOrders));

for ih = 1:length(hVec)
    h = hVec(ih);
    tout = 0:h:5;

    if usePoly
        tpts = linspace(0, tout(end), nWayPoints);
        [q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tout);
    else
        q   = fun_f(tout);
        qd  = fun_f_dt(tout);
        qdd = fun_f_ddt(tout);
    end

    for iO = 1:numel(FDOrders)
        [qd_fd{iO}, qdd_fd{iO}] = diffHigherOrder(q, h, FDOrders(iO));
        error_qd(ih,iO) = norm(abs(qd - qd_fd{iO}))*sqrt(h);
        error_qdd(ih,iO) = norm(abs(qdd - qdd_fd{iO}))*sqrt(h);
    end
end


%% Plot errors

figure("Name", "Convergence error qd");
loglog(hVec, error_qd, '-o');
grid on;
legend("1st order", "2nd order", "4th order", "interpreter", "latex", "Location","east")

hold on;
for iO = 1:numel(FDOrders)
    coeff = error_qd(1,iO) / hVec(1)^FDOrders(iO);
    loglog(hVec, (coeff * hVec.^FDOrders(iO)), '--', "DisplayName", sprintf("$%.2f h^{%.1f}$", coeff, FDOrders(iO)));
end
colororder(lines(numel(FDOrders)));

title("Error qd")
xlabel("time step $h$ in s", "Interpreter", "latex");

figure("Name", "Convergence error qdd");
loglog(hVec, error_qdd, '-o')
grid on;
legend("1st order", "2nd order", "4th order", "interpreter", "latex", "Location","east")

hold on;
for iO = 1:numel(FDOrders)
    coeff = error_qdd(1,iO) / hVec(1)^FDOrders(iO);
    loglog(hVec, (coeff * hVec.^FDOrders(iO)), '--', "DisplayName", sprintf("$%.2f h^{%.1f}$", coeff, FDOrders(iO)));
end
colororder(lines(numel(FDOrders)));

title("Error qdd")
xlabel("time step $h$ in s", "Interpreter", "latex");
