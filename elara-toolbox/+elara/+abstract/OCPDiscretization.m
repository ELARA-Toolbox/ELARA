classdef (Abstract) OCPDiscretization
    %% Abstract class defining a discretization scheme for an OCP
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties(Abstract,SetAccess=private)
        timeStepStageValues (:,1) double
    end
    properties(Abstract,Constant)
        % Defines the discretization type
        type           (1,1) string {mustBeMember(type, ["varint", "ode"])}
    end

    methods (Abstract)
        % Methods to integrate a (vector-valued) variable over the OCP time
        % horizon as part of the cost function
        integrateCostFunctionValue(obj, OCP, x)
        integrateCostFunctionValueSpline(obj, OCP, x, x_C)
    end
end
