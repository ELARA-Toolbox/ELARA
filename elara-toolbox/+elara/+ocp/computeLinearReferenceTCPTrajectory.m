function x_TCP_traj = computeLinearReferenceTCPTrajectory(OCP, opts)
    %% Generate desired TCP trajectory: Linear Trajectory in R3 from x0 to xF
    % as a jerk-minimal, high-order polynomial
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        OCP     (1,1) elara.ocp.Problem

        % Pre- and post-actuation times
        % (where the trajectory is kept constant)
        opts.tPreAct    (1,1) double = 0;
        opts.tPostAct   (1,1) double = 0;
    end
    arguments (Output)
        x_TCP_traj  (3,:) double
    end

    % Initial position: TCP position in initial configuration
    g0 = OCP.system.computeFwdKin(OCP.q0);
    g_TCP_0 = g0(OCP.system.indexTCPFrame).mat*OCP.system.g_B_TCP;
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
end
