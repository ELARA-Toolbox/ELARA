function [x_TCP_traj, fhs] = generateDesiredTCPTrajLinear(MBSim, OCP, opts)
    %% Generate desired TCP trajectory: Linear Trajectory in R3 from x0 to xF
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        MBSim   (1,1) MBSimulation
        OCP     (1,1) OCPDefinition

        % Pre- and post-actuation times
        % (where the trajectory is kept constant)
        opts.tPreAct    (1,1) double = 0;
        opts.tPostAct   (1,1) double = 0;

        % Generate plots?
        opts.generatePlots (1,1) logical = true;
    end
    arguments (Output)
        x_TCP_traj  (3,:) double
        fhs         (:,1)
    end

    % Initial position: TCP position in initial configuration
    g0 = MBSim.MBSys.computeFwdKin(OCP.q0);
    g_TCP_0 = g0(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
    x_TCP_0 = g_TCP_0(1:3, 4);

    % Generate trajectory
    xpts = [x_TCP_0, OCP.x_TCP_F];
    tpts = [OCP.tout(1) + opts.tPreAct; OCP.tout(end)-opts.tPostAct];
    x_TCP_traj_dyn = minjerkpolytraj(xpts, tpts, round((tpts(2)-tpts(1))/OCP.h)+1);

    x_TCP_traj = [
        repmat(x_TCP_0, [1, round((tpts(1)-OCP.tout(1))/OCP.h)]), ...
        x_TCP_traj_dyn, ...
        repmat(OCP.x_TCP_F, [1, round((OCP.tout(end)-tpts(2))/OCP.h)]), ...
        ];


    %% Plotting

    if opts.generatePlots
        [x_TCP_traj_dt, x_TCP_traj_ddt] = diff4thOrder(x_TCP_traj, OCP.h);

        if OCP.Name == ""
            figPrefix = "";
        else
            figPrefix = strcat(OCP.Name, ": ");
        end

        fhs(1) = figure("Name", strcat(figPrefix, " TCP Trajectory Desired"), "NumberTitle", "off");
        tiledlayout("vertical");

        nexttile;
        plot(OCP.tout, x_TCP_traj);
        grid on;
        ylabel("$x_{TCP}$ in m", "Interpreter", "latex")
        xlabel("$t$ in s", "Interpreter", "latex")
        legend("$x$", "$y$", "$z$", "interpreter", "latex");
        title("Desired TCP trajectory");

        nexttile;
        plot(OCP.tout, x_TCP_traj_dt);
        grid on;
        ylabel("$\dot{x}_{TCP}$ in m/s", "Interpreter", "latex")
        xlabel("$t$ in s", "Interpreter", "latex")

        nexttile;
        plot(OCP.tout, x_TCP_traj_ddt);
        grid on;
        ylabel("$\ddot{x}_{TCP}$ in m/s/s", "Interpreter", "latex")
        xlabel("$t$ in s", "Interpreter", "latex")

        % 3D visualization
        fhs(2) = figure("Name", strcat(figPrefix, "TCP Trajectory Vis"), "NumberTitle", "off");
        init3Dplot("createFigure",false);
        MBSim.visualizeSystemConfig(OCP.q0, "createFigure", false);
        plot3(x_TCP_traj(1,:), x_TCP_traj(2,:), x_TCP_traj(3,:), "-o");

        xlim([ ...
            min([0, min(x_TCP_traj(1,:))])-0.1, ...
            max([0, max(x_TCP_traj(1,:))])+0.1, ...
            ]);
        ylim([ ...
            min([0, min(x_TCP_traj(2,:))])-0.1, ...
            max([0, max(x_TCP_traj(2,:))])+0.1, ...
            ]);
        zlim([ ...
            min([0, min(x_TCP_traj(3,:))]), ...
            max([0.8, max(x_TCP_traj(3,:))])+0.1, ...
            ]);
    else
        fhs = gobjects(0);
    end
end