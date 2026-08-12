function [q_init, q_dot_init, u_init, MBSim, qF, uF] = computeInitialGuessInvDyn(OCP, opts)
    %% Generate an OCP initial guess using inverse dynamics
    %
    % Method:
    % 1. Compute a static configuration and controls for the desired TCP
    %    position.
    % 2. Compute a smooth configuration trajectory from the initial to the
    %    final configuration.
    % 3. Compute the corresponding control trajectory by inverse dynamics.
    arguments
        OCP     (1,1) elara.ocp.Problem

        opts.doIDForwardSim (1,1) logical = false;

        % Sample time used for trajectory generation, inverse dynamics,
        % and the optional forward simulation. It must divide the OCP time
        % horizon into an integer number of intervals.
        opts.h              (1,1) double {mustBePositive} = OCP.h;

        % Method to compute inverse dynamics
        % DEL: With DEL/variational integrator (discrete-time)
        % ODE: With ODE (continuous-time)
        opts.invDynMethod   (1,1) string {mustBeMember(opts.invDynMethod, ["DEL", "ODE"])} = "DEL";

        opts.createDebugPlots (1,1) logical = true;
    end

    fprintf("\nGenerating initial guess.\n");
    tIGStart = tic;

    systemNum = OCP.systemNum;
    systemSym = OCP.systemSym;
    simPars = OCP.simPars;

    % A simulation object is only needed as a result/visualization
    % container and for the optional forward simulation.
    MBSim = OCP.getSimulationObject;

    if isempty(OCP.qF)
        OCP_stat = OCP;

        if ~isempty(OCP.x_TCP_waypoints)
            nWPts = size(OCP.x_TCP_waypoints,2);
            x_TCP_waypoints = OCP.x_TCP_waypoints;
        else
            nWPts = 1;
            x_TCP_waypoints = OCP.x_TCP_F;
        end
        qStat = zeros(systemNum.nDoF, nWPts);
        uStat = zeros(systemNum.nInputs, nWPts);

        for iWpt = 1:nWPts
            %% Compute an optimal static equilibrium
            OCP_stat.x_TCP_F = x_TCP_waypoints(:,iWpt);

            fprintf("Computing optimal steady-state configuration...\n\n");

            [qF, uF] = elara.ocp.computeStaticEquilibrium(OCP_stat);

            fprintf("\nComputation time static optimization: %f s\n\n", toc(tIGStart));
            disp("Computed static controls (N or Nm):")
            disp(uF.');

            % For revolute joints: Remove offsets by 2pi
            % TODO: This line is only valid for revolute joints! For any other
            % screw joint (prismatic or screw), it produces wrong values
            jointIndices = systemSym.frames.qIndices(1, systemSym.frames.jointType==1);
            qF(jointIndices) = wrapToPi(qF(jointIndices));

            gOptStatic = systemNum.computeFwdKin(qF);
            g_TCP = gOptStatic(:,:,systemNum.indexTCPFrame)*systemNum.g_B_TCP;
            fprintf("Distance desired TCP position:     %.2e m\n", ...
                norm(OCP_stat.x_TCP_F - g_TCP(1:3, 4)));

            qStat(:,iWpt) = qF;
            uStat(:,iWpt) = uF;
        end

    else
        % Final configuration given instead of desired TCP position
        qStat = OCP.qF;
        uStat = zeros(systemNum.nInputs, 1);
    end
    qF = qStat(:,end);
    uF = uStat(:,end);


    %% Compute configuration trajectory

    % Simulation settings (inverse dynamics)
    % Include both interval endpoints so the final step can be evaluated
    % consistently.
    h_ID = opts.h;
    nSteps_ID = round(OCP.tEnd/h_ID);
    tout_ID = (0 : h_ID : h_ID*nSteps_ID).';

    if isempty(OCP.qDot0)
        qDot0 = zeros(systemNum.nDoF,1);
    else
        qDot0 = OCP.qDot0;
    end

    if isempty(OCP.qDotF)
        qDotF = zeros(systemNum.nDoF,1);
    else
        qDotF = OCP.qDotF;
    end

    if isempty(OCP.x_TCP_waypoints)
        qpts = [OCP.q0, qStat];
        qdpts = [qDot0, qDotF];

        tpts = [OCP.tout(1) + OCP.tPreAct; OCP.tout(end)-OCP.tPostAct];

        assert(tpts(1) < tpts(2), ...
            "The pre- and post-actuation durations must leave a positive trajectory interval.");

        [q_init_dyn, q_dot_init_dyn, q_ddot_init_dyn] = minjerkpolytraj( ...
            qpts, tpts, round((tpts(2)-tpts(1))/h_ID) + 1, ...
            "VelocityBoundaryCondition", qdpts);

        q_init = [
            repmat(OCP.q0, [1, round((tpts(1)-OCP.tout(1))/h_ID)]), ...
            q_init_dyn, ...
            repmat(qF, [1, round((OCP.tout(end)-tpts(2))/h_ID)]), ...
            ];
        q_dot_init = [
            repmat(qDot0, [1, round((tpts(1)-OCP.tout(1))/h_ID)]), ...
            q_dot_init_dyn, ...
            repmat(qDotF, [1, round((OCP.tout(end)-tpts(2))/h_ID)]), ...
            ];
        q_ddot_init = [
            zeros(systemNum.nDoF, round((tpts(1)-OCP.tout(1))/h_ID)), ...
            q_ddot_init_dyn, ...
            zeros(systemNum.nDoF, round((OCP.tout(end)-tpts(2))/h_ID)), ...
            ];
    else
        [q_init, q_dot_init, q_ddot_init] = minjerkpolytraj(qStat, ...
            OCP.x_TCP_timepoints, length(OCP.tout));
    end


    if opts.createDebugPlots
        % Visualize the initial and final configurations
        MBSim.visualizeSystemConfig(OCP.q0, "figureName", "Vis. Initial Config");
        title("Initial Configuration")
        MBSim.visualizeSystemConfig(qF, "figureName", "Vis. Final Config");
        title("Final Configuration")
        elara.visualization.CoordinateFrame( ...
            elara.SE3.matrix(eye(3), OCP.x_TCP_F));


        % Plot the generated trajectory
        figure("Name", "Coordinates IG Interp. Trajectory", "NumberTitle", "off");
        tiledlayout("vertical");
        nexttile;
        plot(tout_ID, q_init);
        grid on;
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("$q$", "Interpreter", "latex");
        legend(arrayfun(@(x) sprintf("$q_{%d}$", x), 1:systemNum.nDoF), "Interpreter", "latex");
        xlim([tout_ID(1),tout_ID(end)]);

        nexttile;
        plot(tout_ID, q_dot_init);
        grid on;
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("$\dot{q}$", "Interpreter", "latex");
        legend(arrayfun(@(x) sprintf("$q_{%d}$", x), 1:systemNum.nDoF), "Interpreter", "latex");
        xlim([tout_ID(1),tout_ID(end)]);
    end

    %% Inverse Dynamics

    switch opts.invDynMethod
        case "DEL"
            [uInit_ID, solInfo] = elara.dynamics.num.inverseDynamicsDEL( ...
                systemNum, simPars, q_init, q_dot_init, h_ID, OCP.uMin, OCP.uMax);
        case "ODE"
            [uInit_ID, solInfo] = elara.dynamics.num.inverseDynamicsODE( ...
                systemNum, simPars, q_init, q_dot_init, q_ddot_init, OCP.uMin, OCP.uMax);
        otherwise
    end
    fprintf("Inverse dynamics residual norm: max = %e, mean = %e\n", max(abs(solInfo.resNorm)), mean(abs(solInfo.resNorm)));


    %% Forward simulation

    MBSim.Name = "Initial Guess";

    % Specify simulation parameters
    MBSim.parameters.tEnd  = OCP.tEnd;
    MBSim.parameters.q0    = OCP.q0;
    MBSim.parameters.qDot0 = qDot0;

    % Initial-guess controls
    MBSim.parameters.uSampleTimes  = tout_ID;
    MBSim.parameters.uSampleValues = uInit_ID;

    % Solver settings
    MBSim.integrator = elara.integration.VIBroyden;
    MBSim.integrator.h = h_ID;
    MBSim.integrator.JacobianIterationThreshold = 2;
    MBSim.integrator.tolerance = 1e-8;

    if opts.doIDForwardSim
        MBSim.integrator.useFirstOrderDissipation = false;
    else
        % Use full 2nd-order dissipation (a = 1/2) only for rigid systems and
        % simplified dissipation (rectangle rule, a = 0) for flexible systems for
        % higher stability
        MBSim.integrator.useFirstOrderDissipation = ~all(systemNum.frames.jointType == 1);
    end

    % Start integration
    MBSim = MBSim.simulateSystem;

    if opts.doIDForwardSim
        q_init_HF     = MBSim.results.q;
        q_dot_init_HF = MBSim.results.q_dot;
        tout_HF       = MBSim.results.tout;
    else
        q_init_HF     = q_init;
        q_dot_init_HF = q_dot_init;
        tout_HF       = tout_ID;
    end

    %% Downsample results to OCP time step

    if OCP.h ~= h_ID
        u_init     = interp1(tout_ID, uInit_ID.', OCP.tout, 'pchip').';
        q_init     = interp1(tout_HF, q_init_HF.', OCP.tout, 'pchip').';
        q_dot_init = interp1(tout_HF, q_dot_init_HF.', OCP.tout, 'pchip').';
    else
        u_init     = uInit_ID;
        q_init     = q_init_HF;
        q_dot_init = q_dot_init_HF;
    end

    fprintf("\nOverall computation time initial guess: %f s.\n\n", toc(tIGStart));
end
