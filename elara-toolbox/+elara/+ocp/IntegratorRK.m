classdef IntegratorRK < elara.abstract.OCPDiscretizationODE
    %% Class defining an RK Integrator for an OCP
    properties
        method (1,1) string {mustBeMember(method, ["RK2", "RK4"])} = "RK4";
    end
    properties(SetAccess=private)
        timeStepStageValues
    end
    methods
        function obj = IntegratorRK(method)
            arguments
                method (1,1) string {mustBeMember(method, ["RK2", "RK4"])} = "RK4";
            end
            obj.method = method;
        end
        function stageVals = get.timeStepStageValues(obj)
            [~, ~, stageVals] = getButcherTableau(obj.method);
        end
        function J = integrateCostFunctionValue(obj, OCP, x)
            arguments
                obj

                OCP (1,1) elara.ocp.Problem

                % Matrix with function values to integrate;
                % each column represents one time step
                x   (:,:)
            end
            J = casadi.MX.zeros(1,1);
            [~, bRK, cRK] = getButcherTableau(obj.method);
            for k = 1:OCP.nSteps
                for i = 1:length(bRK) % loop over nr. of stages s
                    % Interpolated RK stage values
                    x_i = (1 - cRK(i)) * x(:,k)   + cRK(i) * x(:,k+1);

                    % Accumulate weighted stage cost
                    J = J{1} + OCP.h * bRK(i) * sumsqr(x_i);
                end
            end

        end
        function J = integrateCostFunctionValueSpline(obj, OCP, ~, x_C)
            arguments
                obj

                OCP (1,1) elara.ocp.Problem

                % Placeholder argument for consistency with class function
                % interface
                ~

                % Cell array with function values to integrate;
                % dimensions (nStages, nSteps)
                x_C   (:,:) cell
            end
            J = casadi.MX.zeros(1,1);
            [~, bRK, ~] = getButcherTableau(obj.method);
            for k = 1:OCP.nSteps
                for i = 1:length(bRK) % loop over nr. of stages s
                    % Accumulate weighted stage cost
                    J{1} = J{1} + OCP.h * bRK(i) * sumsqr(x_C{i, k});
                end
            end
        end
        function eq_int = getIntegrationStepConstraint(obj, FFun, x_kSym, x_k1Sym, u_kSym, u_k1Sym, h)
            arguments
                obj

                % Function object defining the ODE
                FFun        (1,1)

                % State and input variables at time steps k and k+1
                x_kSym      (:,1)
                x_k1Sym     (:,1)
                u_kSym      (:,1)
                u_k1Sym     (:,1)
                % Time step
                h           (1,1)
            end
            % RK discretization with multiple steps per NLP interval
            [A, b, c] = getButcherTableau(obj.method);
            % Todo: Add MultiStep RK function
            % x_curr = x_kSym;
            % for iStep = 1%:OCP.nRKSteps
            %     x_curr = RKStepOCPLinearInputInterp(FFun, x_curr, u_kSym, u_k1Sym, h/OCP.nRKSteps, A, b, c);
            % end
            x_k1 = RKStepOCPLinearInputInterp(FFun, x_kSym, u_kSym, u_k1Sym, h, A, b, c);
            eq_int = x_k1 - x_k1Sym;
        end
        function eq_int = getIntegrationStepConstraintSpline(obj, FFun, x_kSym, x_k1Sym, u_kStageSym, h)
            arguments
                obj

                % Function object defining the ODE
                FFun        (1,1)

                % State variables at time steps k and k+1
                x_kSym      (:,1)
                x_k1Sym     (:,1)

                % Input variables with intermediate stage values,
                % dimensions (nInputs, nStages)
                u_kStageSym (:,:)

                % Time step
                h           (1,1)
            end
            % RK discretization with multiple steps per NLP interval
            [A, b, c] = getButcherTableau(obj.method);
            x_k1 = RKStepOCP(FFun, x_kSym, u_kStageSym, h, A, b, c);
            eq_int = x_k1 - x_k1Sym;
        end
    end
end
