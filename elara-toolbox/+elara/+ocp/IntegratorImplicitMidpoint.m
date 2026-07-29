classdef IntegratorImplicitMidpoint < elara.abstract.OCPDiscretizationODE
    %% Class defining an Implicit Midpoint Rule Integrator for an OCP
    properties(SetAccess=private)
        timeStepStageValues = 0.5;
    end

    methods(Static)
        function J = integrateCostFunctionValue(OCP, x)
            arguments
                OCP (1,1) elara.ocp.Problem

                % Matrix with function values to integrate;
                % each column represents one time step
                x   (:,:)
            end
            x_mid = (x(:,1:end-1) + x(:,2:end))/2;
            J = OCP.h * (sumsqr( x_mid ));
        end
        function J = integrateCostFunctionValueSpline(OCP, ~, x_C)
            arguments
                OCP (1,1) elara.ocp.Problem

                % Placeholder argument for consistency with class function
                % interface
                ~

                % Cell array with function values to integrate;
                % each column represents one time step
                x_C   (:,:) cell
            end
            x_mid = horzcat(x_C{:});

            % Remove duplicate / extra end point in midpoint values
            x_mid = x_mid(:,1:OCP.nSteps);

            J = OCP.h * (sumsqr( x_mid ) );
        end
        function eq_int = getIntegrationStepConstraint(FFun, x_kSym, x_k1Sym, u_kSym, u_k1Sym, h)
            arguments
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
            x_mid = (x_kSym + x_k1Sym)/2;
            u_mid = (u_kSym + u_k1Sym)/2;
            eq_int = x_kSym + h * FFun(x_mid, u_mid) - x_k1Sym;
        end
        function eq_int = getIntegrationStepConstraintSpline(FFun, x_kSym, x_k1Sym, u_kStageSym, h)
            arguments
                % Function object defining the ODE
                FFun        (1,1)

                % State variables at time steps k and k+1
                x_kSym      (:,1)
                x_k1Sym     (:,1)

                % Input variables with intermediate stage values,
                % dimensions (nInputs, nStages)
                u_kStageSym (:,1)

                % Time step
                h           (1,1)
            end
            x_mid = (x_kSym + x_k1Sym)/2;
            eq_int = x_kSym + h * FFun(x_mid, u_kStageSym) - x_k1Sym;
        end
    end
end
