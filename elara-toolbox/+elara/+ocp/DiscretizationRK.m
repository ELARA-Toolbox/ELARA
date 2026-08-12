classdef DiscretizationRK < elara.abstract.OCPDiscretizationODE
    %% Class defining an RK discretization for an OCP
    properties
        method (1,1) string {mustBeMember(method, ["RK2", "RK4"])} = "RK4";
    end
    properties(SetAccess=private)
        timeStepStageValues
    end
    methods
        function obj = DiscretizationRK(method)
            arguments
                method (1,1) string {mustBeMember(method, ["RK2", "RK4"])} = "RK4";
            end
            obj.method = method;
        end
        function stageVals = get.timeStepStageValues(obj)
            [~, ~, stageVals] = elara.internal.ocp.butcherTableau(obj.method);
        end
        function J = integrateCostFunctionValue(obj, OCP, x)
            arguments
                obj

                OCP (1,1) elara.ocp.Problem

                % Matrix with function values to integrate;
                % each column represents one time node
                x   (:,:)
            end
            J = casadi.MX.zeros(1,1);
            [~, bRK, cRK] = elara.internal.ocp.butcherTableau(obj.method);
            for k = 1:OCP.nSteps
                for i = 1:length(bRK) % Loop over Runge-Kutta stages
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
                % dimensions (nStages, nSteps+1)
                x_C   (:,:) cell
            end
            J = casadi.MX.zeros(1,1);
            [~, bRK, ~] = elara.internal.ocp.butcherTableau(obj.method);
            for k = 1:OCP.nSteps
                for i = 1:length(bRK) % Loop over Runge-Kutta stages
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

                % State and control variables at time nodes k and k+1
                x_kSym      (:,1)
                x_k1Sym     (:,1)
                u_kSym      (:,1)
                u_k1Sym     (:,1)
                % Time step
                h           (1,1)
            end
            % Apply one Runge-Kutta step per NLP interval
            [A, b, c] = elara.internal.ocp.butcherTableau(obj.method);
            x_k1 = elara.internal.ocp.explicitRungeKuttaStepLinearInput( ...
                FFun, x_kSym, u_kSym, u_k1Sym, h, A, b, c);
            eq_int = x_k1 - x_k1Sym;
        end
        function eq_int = getIntegrationStepConstraintSpline(obj, FFun, x_kSym, x_k1Sym, u_kStageSym, h)
            arguments
                obj

                % Function object defining the ODE
                FFun        (1,1)

                % State variables at time nodes k and k+1
                x_kSym      (:,1)
                x_k1Sym     (:,1)

                % Control variables at the intermediate stage values,
                % dimensions (nInputs, nStages)
                u_kStageSym (:,:)

                % Time step
                h           (1,1)
            end
            % Apply one Runge-Kutta step per NLP interval
            [A, b, c] = elara.internal.ocp.butcherTableau(obj.method);
            x_k1 = elara.internal.ocp.explicitRungeKuttaStep( ...
                FFun, x_kSym, u_kStageSym, h, A, b, c);
            eq_int = x_k1 - x_k1Sym;
        end
    end
end
