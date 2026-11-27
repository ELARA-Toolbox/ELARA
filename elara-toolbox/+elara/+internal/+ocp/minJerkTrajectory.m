function [q, q_dot, q_ddot] = minJerkTrajectory(qpts, tpts, tout, qDotPts)
    % Evaluate a minimum-jerk polynomial on the requested time grid.
    % Outside the waypoint interval, hold the endpoint configuration constant.
    % Outputs have one row per coordinate and one column per time in tout.
    arguments
        % Waypoint coordinates, with one column per waypoint (e.g., joint
        % coordinates or Cartesian TCP positions).
        qpts    (:,:) double

        % Strictly increasing waypoint times, in seconds.
        tpts    (:,1) double

        % Evaluation times, in seconds; may extend beyond the waypoint times.
        tout    (:,1) double

        % Optional waypoint velocities, with the same size as qpts.
        % Empty uses minjerkpolytraj's default velocity boundary conditions.
        qDotPts (:,:) double = [];
    end

    % Only the piecewise polynomial (pp) output is needed, so request just
    % two samples here and evaluate the polynomial on tout below.
    if isempty(qDotPts)
        [~, ~, ~, ~, pp] = minjerkpolytraj(qpts, tpts, 2);
    else
        [~, ~, ~, ~, pp] = minjerkpolytraj(qpts, tpts, 2, ...
            "VelocityBoundaryCondition", qDotPts);
    end

    % Clamp evaluation times to hold the first and last waypoint positions.
    tEval = min(max(tout.', tpts(1)), tpts(end));
    q = ppval(pp, tEval);
    if nargout > 1
        % Differentiate the pp coefficients directly; no additional toolbox
        % is needed to evaluate velocities and accelerations on the same grid.
        pp.coefs = pp.coefs(:,1:end-1) .* (pp.order-1:-1:1);
        pp.order = pp.order - 1;
        % Held portions have zero velocity and acceleration.
        isHold = tout.' < tpts(1) | tout.' > tpts(end);
        q_dot = ppval(pp, tEval);
        q_dot(:,isHold) = 0;
        if nargout > 2
            pp.coefs = pp.coefs(:,1:end-1) .* (pp.order-1:-1:1);
            pp.order = pp.order - 1;
            q_ddot = ppval(pp, tEval);
            q_ddot(:,isHold) = 0;
        end
    end
end
