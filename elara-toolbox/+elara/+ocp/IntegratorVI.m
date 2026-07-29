classdef IntegratorVI < elara.abstract.OCPDiscretization
    %% Class defining a Variational Integrator for OCP discretization
    properties
        % Generalized trapezoidal rule factor of the interior integration
        % steps (only relevant for dissipation)
        % 0 = Rectangle Rule     (Second order only without dissipation)
        % 1/2 = Trapezoidal rule (Always second order)
        aTrapez   (1,1) double = 0.5;
    end
    properties(SetAccess=private)
        timeStepStageValues = 0;
    end
    properties(Constant)
        % Defines the integrator type
        type = "varint";
    end

    methods (Static)
        function J = integrateCostFunctionValue(OCP, x)
            arguments
                OCP (1,1) elara.ocp.Problem

                % Matrix with function values to integrate;
                % each column represents one time step
                x   (:,:)
            end
            % Trapezoidal rule
            J = OCP.h * (sumsqr(x(:,1))/2 + sumsqr(x(:,2:end-1)) + sumsqr(x(:,end))/2 );
        end
    end
    methods
        function J = integrateCostFunctionValueSpline(obj,OCP, x, ~)
            arguments
                obj

                OCP (1,1) elara.ocp.Problem

                % Matrix with function values to integrate;
                % each column represents one time step
                x   (:,:)

                % Placeholder argument for consistency with class function
                % interface
                ~
            end
            % No special integration with spline values (no intermediate
            % stage values); just use regular integration function
            J = obj.integrateCostFunctionValue(OCP, x);
        end
    end
end
