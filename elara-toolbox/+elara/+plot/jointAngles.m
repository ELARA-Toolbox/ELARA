function figHandle = jointAngles(sim, opts)
    arguments
        sim              (1,1) elara.Simulation
        opts.nameString  (1,1) string = ""
    end
    figHandle = figure( ...
        'Name', strcat(opts.nameString, "JointAngles"), 'NumberTitle','off');

    % Get indices of the joint angles in the coordinate vector
    thetaIndices = sim.system.frames.qIndices(:,sim.system.frames.jointType == 1);

    if  ~isempty(thetaIndices)
        theta     = sim.results.q(thetaIndices(1,:),:);
        theta_dot = sim.results.q_dot(thetaIndices(1,:),:);

        % Plot joint angles
        tiledlayout("vertical");
        nexttile;

        plot(sim.results.tout, rad2deg(theta));
        legend( ...
            arrayfun(@(x) sprintf("$\\theta_%d$", x), 1:size(theta,1)), ...
            "Interpreter","latex");

        xlabel('time $t$ / s', 'interpreter', 'latex')
        ylabel('$\theta_i$ / deg', 'interpreter', 'latex')
        grid on
        box on
        title("Joint Angles over Time", "Interpreter","latex")

        % Plot angular velocities
        nexttile;
        if isempty(theta_dot)
            warning("theta_dot not defined. Using NaN values.")
            theta_dot = nan(size(theta));
        end

        plot(sim.results.tout, rad2deg(theta_dot));
        legend( ...
            arrayfun(@(x) sprintf("$\\dot{\\theta}_%d$", x), 1:size(theta,1)), ...
            "Interpreter","latex");

        xlabel('time $t$ / s', 'interpreter', 'latex')
        ylabel('$\dot{\theta}_i$ / deg / s', 'interpreter', 'latex')
        grid on
        box on
        title("Joint Angular Velocity over Time", "Interpreter","latex")
    end
end
