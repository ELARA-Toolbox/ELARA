function [q_k1, eta_k, g_rel_k1, H_k, solData] = solveImplicitDELBroyden( ...
        system, q_k, q_k0, g_rel_k, H_k, updateInvJacobian, forceSolverIteration, ...
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
        system  (1,1) elara.SystemNum

        % Generalized coordinates at the current time step (nDoF, 1)
        q_k     (:,1) double

        % Generalized coordinates at the previous time step (nDoF, 1)
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

        % External frame forces (wrenches in se(3)*) at the current time
        % step, (6, nFrames)
        f_frame_k_b     (6,:) double   % Wrench in the local / body frame

        % Generalized forces (nDoF, 1)
        f_gen_k         (:,1) double

        % Solver configuration
        solverConfig    (1,1) elara.internal.integration.VIBroydenConfig

        % Weighting factor in the generalized trapezoidal rule
        % Rectangle rule: a = 0, trapezoidal rule: a = 1/2
        a               (1,1) double
    end
    h = solverConfig.h;

    % Compute geometric Jacobian
    JGeom_k = system.computeGeomJacobianFast(q_k, g_rel_k);

    % Initial value for the solution
    % Compute as explicit Euler step as done in [Lee+20, Sec.3.3], IG2
    q_k1 = 2*q_k - q_k0;

    % Initialize ExitFlag to 1; will be set to 0 if a solution is found
    solData.ExitFlag = 1;
    solData.ImplicitIterations = solverConfig.maxIterations;


    %% Initial function evaluation
    g_rel_k1 = system.computeJointTransformations(q_k1);
    eta_k    = system.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);
    resDEL = MBSDynamics_DEL_implicitFun( ...
        system, JGeom_k, h, f_frame_k_b, f_gen_k, eta_k, q_k1-q_k, a ...
        );

    %% Solver loop
    resNorm = norm(resDEL);
    if resNorm > solverConfig.tolerance || forceSolverIteration

        % Update Implicit Jacobian Matrix if Necessary
        if updateInvJacobian
            % Compute mass matrix and absolute dissipation term
            MBeam = zeros(system.nDoF);
            for iFrm = 1:system.nFrames
                MBeam = MBeam ...
                    + JGeom_k(:,:,iFrm).' * (...
                    + elara.SE3.dcayInv( eta_k(:, iFrm) * h ).' * system.frames.MGen(:,:,iFrm) ...
                    ) * JGeom_k(:,:,iFrm);
            end
            % Add Jacobian term due to linear strain-rate dissipation and
            % invert matrix
            % NOTE: For the exact (generalized) trapezoidal rule, the matrix 
            % D is multiplied with (1-a); however, using the "full" D seems
            % to greatly improve convergence
            H_k = inv( MBeam/h + (1)*diag(system.dSys) );
        end

        for iIteration = 1:solverConfig.maxIterations
            % Variables from last solver iteration
            q_k1_l0 = q_k1;
            resDEL_l0 = resDEL;

            % Apply state update
            q_k1 = q_k1_l0 - H_k * resDEL;

            % Compute new residual
            g_rel_k1 = system.computeJointTransformations(q_k1);
            eta_k    = system.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);
            resDEL = MBSDynamics_DEL_implicitFun( ...
                system, JGeom_k, h, f_frame_k_b, f_gen_k, eta_k, q_k1-q_k, a ...
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
        system, JGeom, h, f_frame_k, f_gen_k, eta_k, q_k_diff, a) %#codegen
    %% Evaluate implicit part of the DEL Equations
    arguments (Input)
        system       (1,1) elara.abstract.System

        % Array of Beam Jacobians (6, nDoF, nNodes)
        JGeom       (6, :, :) double

        % Time step
        h           (1,1) double

        % Array of body-fixed forces acting on the frames (6,nFrames)
        f_frame_k   (6,:) double

        % Generalized forces (nDoF, 1)
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
    if any(system.dSys)
        DEL_res_k = f_gen_k + (1-a)*system.dSys .* q_k_diff;
    else
        DEL_res_k = f_gen_k;
    end

    % Evaluate DEL / Compute Residual
    for iFrm = 1:system.nFrames
        DEL_res_k = DEL_res_k ...
            + JGeom(:, :, iFrm).' *( ...
            + elara.SE3.dcayInv(eta_k(:,iFrm)*h).' * system.frames.MGen(:,:,iFrm) * eta_k(:,iFrm) ...
            + f_frame_k(:, iFrm) ...
            ... % Quadratic dissipation in absolute velocities
            ... %+ h* discPars.dQuad .* eta_k(:,iN-1).^2 .* sign(eta_k(:,iN-1)) ...
            );
    end
end
