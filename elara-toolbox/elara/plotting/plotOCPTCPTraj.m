function fh = plotOCPTCPTraj(MBSim, OCP, q_sol)
    %% Plot TCP Trajectory of OCP Solution
    %   Detailed explanation goes here
    arguments (Input)
        MBSim   (1,1) MBSimulation
        OCP     (1,1) OCPDefinition
        q_sol   (:,:) double
    end

    % Compute TCP trajectory
    g_TCP_sol = zeros(4,4, OCP.nSteps+1);
    for iStep = 1:OCP.nSteps+1
        g_k = MBSim.MBSys.computeFwdKin(q_sol(:,iStep));
        g_TCP_sol(:,:,iStep) = g_k(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
    end
    x_TCP_sol = squeeze(g_TCP_sol(1:3,4,:));

    % Plot TCP Trajectory
    if OCP.Name == ""
        figName = "TCP Trajectory Solution";
    else
        figName = strcat(OCP.Name, ": TCP Trajectory Solution");
    end

    fh = figure("Name", figName, "NumberTitle", "off");
    tiledlayout("vertical");

    nexttile;
    plot(OCP.tout, OCP.x_TCP_traj, "-o", "DisplayName", "Desired");
    hold on;
    plot(OCP.tout, x_TCP_sol, "--x", "DisplayName", "Solution");
    grid on;
    ylabel("$x_{TCP}$ in m", "Interpreter", "latex")
    xlabel("$t$ in s", "Interpreter", "latex")
    legend("interpreter", "latex");
    title("Desired TCP trajectory");
    colororder(lines(3));

    nexttile;
    semilogy(OCP.tout, abs(OCP.x_TCP_traj-x_TCP_sol), "-o", "DisplayName", "Desired");
    grid on;
    ylabel("$x_{TCP}$ in m", "Interpreter", "latex")
    xlabel("$t$ in s", "Interpreter", "latex")
    legend("x", "y", "z", "interpreter", "latex", "Location","best");
    title("TCP trajectory error");
end