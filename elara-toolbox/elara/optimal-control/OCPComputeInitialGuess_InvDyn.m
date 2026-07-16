function [q_init, qd_init, u_init, MBSim, qF, uF] = OCPComputeInitialGuess_InvDyn(MBSim, OCP, opts)
    %% Generate Initial Guess for Optimal Control Problem with Inv. Dynamics
    %
    % Method:
    % 1. Compute static inputs for desired TCP position
    % 2. Compute smooth trajectory for q from initial to final
    %    configuration
    % 3. Compute inputs with inverse dynamics
    arguments
        MBSim   (1,1) elara.Simulation
        OCP     (1,1) OCPDefinition

        opts.doIDForwardSim (1,1) logical = false;
        opts.h              (1,1) double  = OCP.h;

        % Method to compute inverse dynamics
        % DEL: With DEL/variational integrator (discrete-time)
        % ODE: With ODE (continuous-time)
        opts.invDynMethod   (1,1) string {mustBeMember(opts.invDynMethod, ["DEL", "ODE"])} = "DEL";

        opts.createDebugPlots (1,1) logical = true;
    end

    fprintf("\nGenerating initial guess.\n");
    tIGStart = tic;

    if isempty(OCP.qF)
        OCP_stat = OCP;

        if ~isempty(OCP.x_TCP_waypoints)
            nWPts = size(OCP.x_TCP_waypoints,2);
            x_TCP_waypoints = OCP.x_TCP_waypoints;
        else
            nWPts = 1;
            x_TCP_waypoints = OCP.x_TCP_F;
        end
        qStat = zeros(MBSim.system.nDoF, nWPts);
        uStat = zeros(MBSim.system.nInputs, nWPts);

        for iWpt = 1:nWPts
            %% Compute optimal steady-state inputs
            OCP_stat.x_TCP_F = x_TCP_waypoints(:,iWpt);

            fprintf("Computing optimal steady state configuration...\n\n");

            MBSysSym = systemNum2SystemSym(MBSim.system);

            [qF, uF] = computeOptimalSteadyStateInputsTCPPos(MBSysSym, OCP_stat, MBSim.parameters);

            fprintf("\nComputation time static optimization: %f s\n\n", toc(tIGStart));
            disp("Computed static inputs (N/Nm):")
            disp(uF.');

            % For revolute joints: Remove offsets by 2pi
            % TODO: This line is only valid for revolute joints! For any other
            % screw joint (prismatic or screw), it produces wrong values
            jointIndices = MBSysSym.frames.qIndices(1, MBSysSym.frames.jointType==1);
            qF(jointIndices) = wrapToPi(qF(jointIndices));

            gOptStatic = MBSim.system.computeFwdKin(qF);
            g_TCP = gOptStatic(:,:,MBSim.system.indexTCPFrame)*MBSim.system.g_B_TCP;
            fprintf("Distance desired TCP position:     %.2e m\n", ...
                norm(OCP_stat.x_TCP_F - g_TCP(1:3, 4)));

            qStat(:,iWpt) = qF;
            uStat(:,iWpt) = uF;
        end

    else
        % Final configuration given instead of desired TCP position
        qStat = OCP.qF;
        uStat = zeros(MBSim.system.nInputs, 1);
    end
    qF = qStat(:,end);
    uF = uStat(:,end);


    %% Compute coordinate trajectory

    % Simulation settings (inverse dynamics)
    % Add one step to the time vector of the inverse dynamics to be able to
    % compute the last step consistently
    h_ID = opts.h;
    nSteps_ID = round(OCP.tF/h_ID);
    tout_ID = (0 : h_ID : h_ID*nSteps_ID).';

    if isempty(OCP.qDot0)
        qDot0 = zeros(MBSim.system.nDoF,1);
    else
        qDot0 = OCP.qDot0;
    end

    if isempty(OCP.qDotF)
        qDotF = zeros(MBSim.system.nDoF,1);
    else
        qDotF = OCP.qDotF;
    end

    if isempty(OCP.x_TCP_waypoints)
        qpts = [OCP.q0, qStat];
        qdpts = [qDot0, qDotF];

        tpts = [OCP.tout(1) + OCP.tPreAct; OCP.tout(end)-OCP.tPostAct];

        [q_init_dyn, qd_init_dyn, qdd_init_dyn] = minjerkpolytraj( ...
            qpts, tpts, round((tpts(2)-tpts(1))/OCP.h) + 1, ...
            "VelocityBoundaryCondition", qdpts);

        q_init = [
            repmat(OCP.q0, [1, round((tpts(1)-OCP.tout(1))/OCP.h)]), ...
            q_init_dyn, ...
            repmat(qF, [1, round((OCP.tout(end)-tpts(2))/OCP.h)]), ...
            ];
        qd_init = [
            repmat(qDot0, [1, round((tpts(1)-OCP.tout(1))/OCP.h)]), ...
            qd_init_dyn, ...
            repmat(qDotF, [1, round((OCP.tout(end)-tpts(2))/OCP.h)]), ...
            ];
        qdd_init = [
            zeros(MBSim.system.nDoF, round((tpts(1)-OCP.tout(1))/OCP.h)), ...
            qdd_init_dyn, ...
            zeros(MBSim.system.nDoF, round((OCP.tout(end)-tpts(2))/OCP.h)), ...
            ];
    else
        [q_init, qd_init, qdd_init] = minjerkpolytraj(qStat, ...
            OCP.x_TCP_timepoints, length(OCP.tout));
    end


    if opts.createDebugPlots
        % Visualize initial and final config
        MBSim.visualizeSystemConfig(OCP.q0, "figureName", "Vis. Initial Config");
        title("Initial Configuration")
        MBSim.visualizeSystemConfig(qF, "figureName", "Vis. Final Config");
        title("Final Configuration")
        coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));


        % Plot generated trajectory
        figure("Name", "Coordinates IG Interp. Trajectory", "NumberTitle", "off");
        tiledlayout("vertical");
        nexttile;
        plot(tout_ID, q_init);
        grid on;
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("$q$", "Interpreter", "latex");
        legend(arrayfun(@(x) sprintf("$q_{%d}$", x), 1:MBSim.system.nDoF), "Interpreter", "latex");
        xlim([tout_ID(1),tout_ID(end)]);

        nexttile;
        plot(tout_ID, qd_init);
        grid on;
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("$\dot{q}$", "Interpreter", "latex");
        legend(arrayfun(@(x) sprintf("$q_{%d}$", x), 1:MBSim.system.nDoF), "Interpreter", "latex");
        xlim([tout_ID(1),tout_ID(end)]);
    end

    %% Inverse Dynamics

    switch opts.invDynMethod
        case "DEL"
            [uInit_ID, solInfo] = computeInverseDynamicsDEL(MBSim, q_init, qd_init, h_ID, OCP.uMin, OCP.uMax);
        case "ODE"
            [uInit_ID, solInfo] = computeInverseDynamicsODE(MBSim, q_init, qd_init, qdd_init, OCP.uMin, OCP.uMax);
        otherwise
    end
    fprintf("Inverse dynamics residual norm: max = %e, mean = %e\n", max(abs(solInfo.resNorm)), mean(abs(solInfo.resNorm)));


    %% Forward simulation

    MBSim.Name = "Initial Guess";

    % Specify Simulation Parameters
    MBSim.parameters.tEnd  = OCP.tF;
    MBSim.parameters.q0    = OCP.q0;
    MBSim.parameters.qDot0 = OCP.qDot0;

    % Initial guess inputs
    MBSim.parameters.uSampleTimes  = tout_ID;
    MBSim.parameters.uSampleValues = uInit_ID;

    % Solver settings
    MBSim.integrator = elara.integration.VIBroyden;
    MBSim.integrator.h = h_ID;
    MBSim.integrator.JacobianIterationThreshold = 2;
    MBSim.integrator.errorMargin = 1e-8;

    if opts.doIDForwardSim
        MBSim.integrator.aTrapez = 1/2;
    else
        % Use full 2nd-order dissipation (a = 1/2) only for rigid systems and
        % simplified dissipation (rectangle rule, a = 0) for flexible systems for
        % higher stability
        MBSim.integrator.aTrapez = 1/2 * all(MBSim.system.frames.jointType == 1);
    end

    % Start integration
    MBSim = MBSim.simulateSystem;

    if opts.doIDForwardSim
        q_init_HF = MBSim.results.q;
        tout_HF   = MBSim.results.tout;
    else
        q_init_HF = q_init;
        tout_HF   = tout_ID;
    end

    %% Downsample results to OCP time step

    if OCP.h ~= h_ID
        u_init = interp1(tout_ID, uInit_ID.', OCP.tout, 'pchip').';
        q_init = interp1(tout_HF, q_init_HF.', OCP.tout, 'pchip').';
    else
        u_init = uInit_ID;
        q_init = q_init_HF;
    end

    fprintf("\nOverall computation time initial guess: %f s.\n\n", toc(tIGStart));
end