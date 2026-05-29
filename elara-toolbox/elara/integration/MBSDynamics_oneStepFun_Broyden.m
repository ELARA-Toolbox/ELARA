function [g_k1, eta_k, q_k1, solData, H_k, g_rel_k1] = MBSDynamics_oneStepFun_Broyden( ...
        MBSys, g_k, q_k, eta_k0, q_k0, g_rel_k,...
        simPars, H_k, updateInvJacobian, forceSolverIteration, ...
        f_frame_k_b, f_frame_k_s, ...
        solverConfig) %#codegen
    %% Variational integrator function to integrate over one timestep
    % from k -> k1.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Multibody system
        MBSys       (1,1) MBSystem

        % Array of SE3 frame configuration matrices at current time step k (4, 4, nFrames)
        g_k         (4,4,:) double

        % Vector of generalized coordinates at current time step (1,nDoF)
        q_k         (:,1) double

        % Array of discrete velocities at previous time step k0 (6, nFrames)
        eta_k0      (6,:) double

        % % Vector of generalized coordinates at last time step (1, nDoF)
        q_k0        (:,1) double

        % Array of update matrices for all frames (4,4,nFrames)
        g_rel_k     (4,4,:) double

        % beamSimPars object containing the simulation parameters
        simPars     (1,1) MBSimPars

        % Initial approximation of the inverse Jacobian matrix for the
        % implicit equation system
        H_k         (:,:) double

        % If true, compute the (inverse) Jacobian instead of using the
        % given inverse Jacobian H_k
        updateInvJacobian (1,1) logical

        % Force a solver iteration by ignoring the error threshold
        forceSolverIteration (1,1) logical

        % External node forces (wrenches in se3*) at current time step (6,nNodes)
        f_frame_k_b   (6,:) double   % Wrench in the local / body frame
        f_frame_k_s   (6,:) double   % Wrench in the inertial/spatial frame

        % Struct containing solver configs
        solverConfig (1,1) varIntSolverConfig
    end

    %% Frame Forces
    h = solverConfig.h;

    % Get gravity and external spatial forces transformed to the body-fixed
    % frames
    f_frame_k_b = -h*f_frame_k_b + h*computeBodyfixedFrameForces(g_k, f_frame_k_s, MBSys, simPars);

    % Add inertia term
    for iFrm = 1:MBSys.nFrames
        f_frame_k_b(:, iFrm) = f_frame_k_b(:, iFrm) ...
            - cayRTDInvSE3(-eta_k0(:,iFrm)*h).' * MBSys.frameData.MGen(:,:,iFrm) * eta_k0(:,iFrm);
    end

    %% Generalized Forces

    % Stresses
    f_gen_k = h*MBSys.cSys .* (q_k - MBSys.qRef);

    % Actuation (if nonzero)
    if ~isempty(simPars.uConst)
        f_gen_k = f_gen_k - h*MBSys.computeInputMatrix(q_k) * simPars.uConst;
    end

    % Linear dissipation in strain rates (for trapezoidal integrator)
    % if any(MBSys.dSys)
    %     f_gen_k = f_gen_k + (MBSys.dSys/2 .* (q_k-q_k0));
    % end

    %% Solve Equation
    a = 1/2;
    [q_k1, eta_k, g_rel_k1, H_k, solData] = solveImplicitDELEquBroyden( ...
        MBSys, q_k, q_k0, g_rel_k, H_k, updateInvJacobian, forceSolverIteration, f_frame_k_b, f_gen_k, ...
        solverConfig, a  ...
        );

    %% Forward Kinematics for the next time step
    g_k1 = MBSys.computeFwdKinFast(g_rel_k1);
end