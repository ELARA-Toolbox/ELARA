classdef (Abstract) OCPIntegratorODE < OCPIntegrator
    %% Abstract class defining an ODE Integrator for an OCP
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