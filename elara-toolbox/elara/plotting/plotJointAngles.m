function figHandle = plotJointAngles(MBSys, simRes, opts)
    arguments
        MBSys           (1,1) elara.internal.System
        simRes          (1,1) MBSimResults
        opts.nameStr    (1,1) string = ""
    end
    figHandle = figure( ...
        'Name', strcat(opts.nameStr, "JointAngles"), 'NumberTitle','off');

    % Get indices of the joint angles in the coordinate vector
    thetaIndices = MBSys.frames.qIndices(:,MBSys.frames.jointType == 1);

    if  ~isempty(thetaIndices)
        theta     = simRes.q(thetaIndices(1,:),:);
        theta_dot = simRes.q_dot(thetaIndices(1,:),:);

        tl = tiledlayout("vertical");
        t = nexttile;

        plot(simRes.tout, rad2deg(theta));
        legend( ...
            arrayfun(@(x) sprintf("$\\theta_%d$", x), 1:size(theta,1)), ...
            "Interpreter","latex");

        xlabel('time $t$ / s', 'interpreter', 'latex')
        ylabel('$\theta_i$ / deg', 'interpreter', 'latex')
        grid on
        box on
        title("Joint Angles over Time", "Interpreter","latex")

        t = nexttile;
        if isempty(theta_dot)
            %theta_dot = gradient(theta) / simPars.h;
            warning("theta_dot not defined. Using NaN values.")
            theta_dot = nan(size(theta));
        end

        plot(simRes.tout, rad2deg(theta_dot));
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
