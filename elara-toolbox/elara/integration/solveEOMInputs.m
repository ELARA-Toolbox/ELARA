function [u, solInfo] = solveEOMInputs(MBSys, res_k, q_k, lb, ub)
    %% Solve (Underactuated) Equations of Motion for System Inputs u
    % with u inside lower and upper bounds lb, ub
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        MBSys (1,1) elara.abstract.System

        % DEL residual at current time step k
        res_k (:,1) double

        % Coordinates at current time step k
        q_k   (:,1) double

        % Upper and lower bounds for u
        lb      (:,1) double
        ub      (:,1) double
    end

    % Coefficient matrix C
    C = -MBSys.computeInputMatrix(q_k);

    % Vector of right-hand-sides
    d = res_k;

    % Add additional column in equation system to minimize input magnitude
    % and remove constant offsets
    CExt = [C; ones(1, MBSys.nInputs)*5e-4];
    dExt = [d;0];

    opts = optimoptions("lsqlin");
    opts.Display = "off";
    opts.Algorithm = "trust-region-reflective";
    opts.FunctionTolerance = 1e-16;
    opts.OptimalityTolerance = 1e-16;
    [u, solInfo.resNorm, solInfo.residual] = lsqlin(CExt, dExt, [], [], [], [], lb, ub, [], opts);
end