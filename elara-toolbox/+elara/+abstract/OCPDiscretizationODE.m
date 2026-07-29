classdef (Abstract) OCPDiscretizationODE < elara.abstract.OCPDiscretization
    %% Abstract class defining an ODE discretization scheme for an OCP
    properties(Constant)
        % Defines the integrator type
        type  = "ode";
    end

    methods (Abstract)
        % Additional abstract functions for the system dynamics step
        % constraints
        getIntegrationStepConstraint
        getIntegrationStepConstraintSpline
    end
end
