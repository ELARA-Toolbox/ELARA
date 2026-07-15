function simResults = integrateMBSDynamics_Broyden(MBSys, simPars, solverConfig) %#codegen
    %% Variational Integrator for a rigid-flexible multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Object defining the multibody system
        MBSys           (1,1) elara.SystemNum

        % simPars object (struct) with simulation parameters.
        % See class definition for details.
        simPars         (1,1) MBSimPars

        % Struct containing solver config
        solverConfig    (1,1) varIntSolverConfig
    end

    %% Validate Simulation Input Data

    assert( numel(simPars.q0) == MBSys.nDoF, ...
        "Vector of initial coordinates has wrong dimensions.");
    assert( solverConfig.h > 0, ...
        "Time step h must be non-zero and positive.");
    assert( simPars.tEnd > 0, ...
        "Simulation end time tEnd must be non-zero and positive.");
    assert( isempty(simPars.uConst) || numel(simPars.uConst) == MBSys.nInputs, ...
        "Vector of constant system inputs has wrong dimensions.");
    assert( isempty(simPars.uSampleValues) || size(simPars.uSampleValues,1) == MBSys.nInputs, ...
        "Nr. of rows of the matrix of time-varying system inputs does not match the nr. of system inputs.");
    assert( isempty(simPars.extWrench_b.wrench) || size(simPars.extWrench_b.wrench,2) == MBSys.nFrames, ...
        "Nr. of columns of the matrix of body-fixed wrenches does not match the nr. of frames.");
    assert( isempty(simPars.extWrench_s.wrench) || size(simPars.extWrench_s.wrench,2) == MBSys.nFrames, ...
        "Nr. of columns of the matrix of spatial frame forces does not match the nr. of frames.");


    %% Initialize output arrays
    % Note: time dimension (outer loop) should be last index,
    % node dimension (inner loop) should be first;
    % Also put data dimensions first, which makes squeeze/reshape
    % unnecessary

    % Time step / Sample time
    h = solverConfig.h;

    % Nr. of integration steps
    nSteps = round( simPars.tEnd / h );

    % Time vector (has length nSteps + 1)
    tout = (0:h:h*nSteps)';

    g     = zeros(4,4, MBSys.nFrames, nSteps+1);   % Configuration
    eta   = zeros(6,   MBSys.nFrames, nSteps+1);   % Discrete velocity
    q     = zeros(MBSys.nDoF, nSteps+1);           % Relative coordinates
    q_dot = zeros(MBSys.nDoF, nSteps+1);           % Relative coordinate velocities

    % Metadata vectors/matrices
    ImplicitError       = nan(1,nSteps+1);
    ImplicitIterations  = nan(1,nSteps+1);
    ExitFlag            = nan(1,nSteps+1);

    % Prepare System Inputs
    u = getIntegratorInputs(MBSys, simPars, tout);


    %% Compute initial step k=1 -> k=2
    % Initial step is computed using the (left) Legendre transform FL- to
    % properly include initial velocities.

    % Weighting factor for the generalized trapezoidal rule:
    % Use full second-order trapezoidal rule for first step for
    % second-order accuracy
    a = 1/2;

    % Initial configuration at k = 1
    q_k = simPars.q0;

    % Initial (continuous-time) momentum
    p_0 = MBSys.computeMassMatrix(q_k) * simPars.qDot0;

    % Forward kinematics for the first step (k = 1)
    [g_0, g_rel_0] = MBSys.computeFwdKin(simPars.q0);

    % Frame forces at first step
    f_frame_0_b = getExternalStepWrenches(simPars.extWrench_b, MBSys.nFrames, 0);
    f_frame_0_s = getExternalStepWrenches(simPars.extWrench_s, MBSys.nFrames, 0);
    
    f_frame_0_b = -h*(1-a)*f_frame_0_b + h*(1-a)*computeBodyfixedFrameForces(g_0, f_frame_0_s, MBSys, simPars);

    % Generalized Forces (stresses, actuation and initial momentum)
    f_gen_0 = h*(1-a)*MBSys.cSys .* (q_k - MBSys.qRef) ...
        - h*(1-a)*MBSys.computeInputMatrixFast(g_rel_0) * u(:,1) ...
        - p_0;

    %%% Solve initial step

    % Force iteration to avoid convergence problems when starting from the
    % equilibrium
    forceSolverIteration = true;
    % Update DEL Jacobian so it can be re-used directly in the loop
    updateInvJacobian    = true;

    q_k0 = q_k; % Only for the initial value of the implicit solver
    [q_k1, eta_k, g_rel_k1, H_k, solData_k] = solveImplicitDELEquBroyden( ...
        MBSys, q_k, q_k0, g_rel_0, zeros(MBSys.nDoF), updateInvJacobian, forceSolverIteration, f_frame_0_b, f_gen_0, ...
        solverConfig, a  ...
        );

    % Forward Kinematics for the second time step (k = 2)
    g_k1 = MBSys.computeFwdKinFast(g_rel_k1);

    % Assign to output arrays
    q(:,1)     = q_k;
    q(:,2)     = q_k1;
    g(:,:,:,1) = g_0;
    g(:,:,:,2) = g_k1;
    eta(:,:,1) = eta_k;
    q_dot(:,1) = (q_k1 - q_k)/h;
    ImplicitError(1)      = solData_k.ImplicitError;
    ImplicitIterations(1) = solData_k.ImplicitIterations;
    ExitFlag(1)           = solData_k.ExitFlag;


    %% Integration loop

    nStepsDone = nSteps;
    forceSolverIteration = false;

    % Use rectangle rule in integration loop for better performance and
    % faster convergences
    aLoop = solverConfig.aTrapez;

    for k = 2:nSteps
        % Check nr. of iterations in the previous time step and recompute
        % Jacobian if necessary
        updateInvJacobian = ImplicitIterations(k-1) > solverConfig.JacobianIterationThreshold;

        % Manage values from last time step
        q_k0 = q_k;
        q_k  = q_k1;
        g_k  = g_k1;
        eta_k0 = eta_k;
        g_rel_k = g_rel_k1;

        % External frame forces from the environment
        f_frame_k_b = getExternalStepWrenches(simPars.extWrench_b, MBSys.nFrames, tout(k));
        f_frame_k_s = getExternalStepWrenches(simPars.extWrench_s, MBSys.nFrames, tout(k));

        % Compute Frame Forces
        % * External frame forces
        % * Gravity and external spatial forces transformed to the body-fixed frames
        % * Inertia term
        f_frame_k_b = ...
            - h*f_frame_k_b ...
            + h*computeBodyfixedFrameForces(g_k, f_frame_k_s, MBSys, simPars) ...
            - computeInertiaTerm(MBSys, eta_k0, h);

        % Compute Generalized Forces (stresses and actuation)
        f_gen_k = h*MBSys.cSys .* (q_k - MBSys.qRef) ...
            - h*MBSys.computeInputMatrixFast(g_rel_k) * u(:,k)...
            + aLoop*MBSys.dSys .* (q_k-q_k0);

        % Solve implicit DEL equation
        [q_k1, eta_k, g_rel_k1, H_k, solData_k] = solveImplicitDELEquBroyden( ...
            MBSys, q_k, q_k0, g_rel_k, H_k, updateInvJacobian, forceSolverIteration, ...
            f_frame_k_b, f_gen_k, solverConfig, aLoop );

        % Forward Kinematics for the next time step
        g_k1 = MBSys.computeFwdKinFast(g_rel_k1);

        % Assign to outputs
        q(:,k+1)     = q_k1;
        g(:,:,:,k+1) = g_k1;
        eta(:,:,k)   = eta_k;
        q_dot(:,k)   = (q_k1 - q_k)/h;
        ImplicitError(k)      = solData_k.ImplicitError;
        ImplicitIterations(k) = solData_k.ImplicitIterations;
        ExitFlag(k)           = solData_k.ExitFlag;

        % Check if solver was successful; cancel simulation if residual is
        % above residual limit
        if ( solData_k.ExitFlag && solData_k.ImplicitError > solverConfig.errorMarginLimit ) ...
                || isnan(solData_k.ImplicitError)
            nStepsDone = k;
            break;
        end
    end

    % Discrete velocities at the final step:
    % Not defined since there is no future time step anymore
    % (velocity at k is the velocity in interval k, k+1)
    eta(:,:,end) = nan(6, MBSys.nFrames);
    q_dot(:,end) = nan(MBSys.nDoF, 1);


    %% Assign to output object
    simResults = MBSimResults;
    simResults.g     = g(:,:,:,1:nStepsDone+1);
    simResults.q     = q(:,1:nStepsDone+1);
    simResults.q_dot = q_dot(:,1:nStepsDone+1);
    simResults.eta   = eta(:,:,1:nStepsDone+1);
    simResults.tout  = tout(1:nStepsDone+1, 1);

    simResults.solverError      = ImplicitError(1:nStepsDone+1);
    simResults.solverIterations = ImplicitIterations(1:nStepsDone+1);
    simResults.solverExitFlag   = ExitFlag(1:nStepsDone+1);
end

function f_inertia_k = computeInertiaTerm(MBSys, eta_k0, h)
    % Compute the inertial term of the DEL equations
    f_inertia_k = zeros(6, MBSys.nFrames);
    for iFrm = 1:MBSys.nFrames
        f_inertia_k(:, iFrm) = cayRTDInvSE3(-eta_k0(:,iFrm)*h).' * MBSys.frameData.MGen(:,:,iFrm) * eta_k0(:,iFrm);
    end
end