function x_TCP_traj = computeLinearReferenceTCPTrajectory(OCP, opts)
    %% Generate a jerk-minimal TCP trajectory in R^3 from x0 to xF
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        OCP     (1,1) elara.ocp.Problem

        % Pre- and post-actuation durations during which the trajectory is
        % held constant
        opts.tPreAct    (1,1) double {mustBeNonnegative,mustBeFinite} = 0;
        opts.tPostAct   (1,1) double {mustBeNonnegative,mustBeFinite} = 0;
    end
    arguments (Output)
        x_TCP_traj  (3,:) double
    end

    % Initial position: TCP position in initial configuration
    systemNum = OCP.systemNum;
    indexTCPFrame = systemNum.indexTCPFrame;
    if ~indexTCPFrame
        indexTCPFrame = systemNum.nFrames;
    end
    g0 = systemNum.computeFwdKin(OCP.q0);
    g_TCP_0 = g0(:,:,indexTCPFrame)*systemNum.g_B_TCP;
    x_TCP_0 = g_TCP_0(1:3, 4);

    % Generate trajectory
    xpts = [x_TCP_0, OCP.x_TCP_F];
    tpts = [OCP.tout(1) + opts.tPreAct; OCP.tout(end)-opts.tPostAct];
    assert(tpts(1) < tpts(2), 'elara:InvalidActuationInterval', ...
        "The pre- and post-actuation durations must leave a positive trajectory interval.");
    x_TCP_traj = elara.internal.ocp.minJerkTrajectory(xpts, tpts, OCP.tout);
end
