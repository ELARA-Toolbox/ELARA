function [q_k1, eta_k, g_rel_k1, H_k, solData] = solveImplicitDELEquBroyden( ...
        MBSys, q_k, q_k0, g_rel_k, H_k, updateInvJacobian, forceSolverIteration, ...
        f_frame_k_b, f_gen_k, solverConfig, a)
    %% Solve implicit part of the DEL function using Broyden's Good Method
    %
    % Created by Maximilian Herrmann
    % Original implementation of Broyden's good method by Philipp Tarbiat
    % (term paper 2021/2022)
    %
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Multibody system
        MBSys   (1,1) elara.abstract.System

        % Vector of generalized coordinates at current time step (1,nDoF)
        q_k     (:,1) double

        % Vector of generalized coordinates at last time step (1, nDoF)
        q_k0    (:,1) double

        % Array of update matrices for all frames (4,4,nFrames)
        g_rel_k (4,4,:) double

        % Initial approximation of the inverse Jacobian matrix for the
        % implicit equation system
        H_k     (:,:) double

        % If true, compute the (inverse) Jacobian instead of using the
        % given inverse Jacobian H_k
        updateInvJacobian       (1,1) logical

        % Force a solver iteration by ignoring the error threshold
        forceSolverIteration    (1,1) logical

        % External node forces (wrenches in se3*) at current time step (6,nNodes)
        f_frame_k_b     (6,:) double   % Wrench in the local / body frame

        % Vector of relative/generalized forces (nDof,1)
        f_gen_k         (:,1) double

        % Struct containing solver configs
        solverConfig    (1,1) varIntSolverConfig

        % Weighting factor in the generalized trapezoidal rule
        % Rectangle rule: a = 0, trapezoidal rule: a = 1/2
        a               (1,1) double
    end
    h = solverConfig.h;

    % Compute geometric Jacobian
    JGeom_k = MBSys.computeGeomJacobianFast(q_k, g_rel_k);

    % Initial value for the solution
    % Compute as explicit Euler step as done in [Lee+20, Sec.3.3], IG2
    q_k1 = 2*q_k - q_k0;

    % Initialize ExitFlag to 1; will be set to 0 if a solution is found
    solData.ExitFlag = 1;
    solData.ImplicitIterations = solverConfig.maxIterations;


    %% Initial function evaluation
    g_rel_k1 = MBSys.computeJointTransformations(q_k1);
    eta_k    = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);
    resDEL = MBSDynamics_DEL_implicitFun( ...
        MBSys, JGeom_k, h, f_frame_k_b, f_gen_k, eta_k, q_k1-q_k, a ...
        );

    %% Solver loop
    resNorm = norm(resDEL);
    if resNorm > solverConfig.tolerance || forceSolverIteration

        % Update Implicit Jacobian Matrix if Necessary
        if updateInvJacobian
            % Compute mass matrix and absolute dissipation term
            MBeam = zeros(MBSys.nDoF);
            for iFrm = 1:MBSys.nFrames
                MBeam = MBeam ...
                    + JGeom_k(:,:,iFrm).' * (...
                    + elara.SE3.dcayInv( eta_k(:, iFrm) * h ).' * MBSys.frames.MGen(:,:,iFrm) ...
                    ) * JGeom_k(:,:,iFrm);
            end
            % Add Jacobian term due to linear strain-rate dissipation and
            % invert matrix
            % NOTE: For the exact (generalized) trapezoidal rule, the matrix 
            % D is multiplied with (1-a); however, using the "full" D seems
            % to greatly improve convergence
            H_k = inv( MBeam/h + (1)*diag(MBSys.dSys) );
        end

        for iIteration = 1:solverConfig.maxIterations
            % Variables from last solver iteration
            q_k1_l0 = q_k1;
            resDEL_l0 = resDEL;

            % Apply state update
            q_k1 = q_k1_l0 - H_k * resDEL;

            % Compute new residual
            g_rel_k1 = MBSys.computeJointTransformations(q_k1);
            eta_k    = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);
            resDEL = MBSDynamics_DEL_implicitFun( ...
                MBSys, JGeom_k, h, f_frame_k_b, f_gen_k, eta_k, q_k1-q_k, a ...
                );

            % Check residual and update H_k
            resNorm = norm(resDEL);
            if resNorm <= solverConfig.tolerance
                % Solution found
                % Set iteration count, exit flag and exit loop
                solData.ExitFlag = 0;
                solData.ImplicitIterations = iIteration;
                break;
            else
                % Update approximation of the jacobian
                s_k = q_k1 - q_k1_l0;
                y_k = resDEL - resDEL_l0;
                H_k = H_k + ((s_k - H_k*y_k) * (s_k.' * H_k)) / (s_k.' * H_k * y_k);
                if any(isnan(H_k))
                    % Exit with solver flag 1 = error
                    break;
                end
            end
        end
    else
        % First evaluation already satisfies tolerance
        solData.ExitFlag = 0;
        solData.ImplicitIterations = 0;
    end

    % Set implicit error
    solData.ImplicitError = resNorm;
end

function DEL_res_k = MBSDynamics_DEL_implicitFun( ...
        MBSys, JGeom, h, f_frame_k, f_gen_k, eta_k, q_k_diff, a) %#codegen
    %% Evaluate implicit part of the DEL Equations
    arguments (Input)
        MBSys       (1,1) elara.abstract.System

        % Array of Beam Jacobians (6, nDoF, nNodes)
        JGeom       (6, :, :) double

        % Time step
        h           (1,1) double

        % Array of body-fixed forces acting on the frames (6,nFrames)
        f_frame_k   (6,:) double

        % Vector of relative/generalized forces (nDof,1)
        f_gen_k     (:,1) double

        % Array of absolute frame velocities in interval (k,k1)
        eta_k       (6,:) double

        % Difference q_k1-q_k of coordinates at steps k+1 and k
        q_k_diff    (:,1) double

        % Weighting factor in the generalized trapezoidal rule
        % Rectangle rule: a = 0, trapezoidal rule: a = 1/2
        a           (1,1) double
    end
    arguments (Output)
        % Residual values of the DEL equations
        DEL_res_k   (:,1) double
    end

    % Add linear dissipation in strain rates
    if any(MBSys.dSys)
        DEL_res_k = f_gen_k + (1-a)*MBSys.dSys .* q_k_diff;
    else
        DEL_res_k = f_gen_k;
    end

    % Evaluate DEL / Compute Residual
    for iFrm = 1:MBSys.nFrames
        DEL_res_k = DEL_res_k ...
            + JGeom(:, :, iFrm).' *( ...
            + elara.SE3.dcayInv(eta_k(:,iFrm)*h).' * MBSys.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            + f_frame_k(:, iFrm) ...
            ... % Quadratic dissipation in absolute velocities
            ... %+ h* discPars.dQuad .* eta_k(:,iN-1).^2 .* sign(eta_k(:,iN-1)) ...
            );
    end
end