classdef BeamParameters
    %% BeamParameters Class storing all parameters defining a geometrically exact beam
    %
    % Maximilian heighterrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Beam Geometry
        height      (1,1) double   % Height
        width      (1,1) double   % Width

        % Mixed parameters
        m       (1,1) double   % Cross-Section mass (per unit length)
        J       (3,3) double   % Cross-Section inertia tensor

        Cgen    (6,6) double   % Stiffness matrix

        Mgen    (6,6) double   % Generalized cross-section mass matrix
        MgenInv (6,6) double

        % Linear strain rate damping coefficient (for all segments)
        % for Kelvin-Voigt type damping
        d       (6,1) double = zeros(6,1);
    end

    methods
        function obj = computeParameters(obj, baseParameters)
            %% Compute Inertia and Stiffness Matrices from "Base Parameters"
            arguments
                obj         (1,1) elara.BeamParameters

                % Struct with required base parameters
                % See below for required fields
                baseParameters  (1,1) struct
            end

            % Assert that the struct has all required fields
            assert(isfield(baseParameters, "E"));   % Young's modulus (N/m^2)
            assert(isfield(baseParameters, "nu"));  % Poisson's number
            assert(isfield(baseParameters, "rho")); % Density (kg/m^3)
            assert(isfield(baseParameters, "A"));   % Cross-section area (m^2)
            assert(isfield(baseParameters, "I_x")); % Second moments of inertia (about x axis of the body-fixed coordinate systems)
            assert(isfield(baseParameters, "I_y")); % Second moments of inertia (about y axis of the body-fixed coordinate systems)
            assert(isfield(baseParameters, "J_P")); % Polar moment of inertia

            % Shear Modulus
            % [LLA11, p.307], also e.g., [Dem+15, p. 80]
            baseParameters.G = baseParameters.E / ( 2 * (1 + baseParameters.nu ) );

            % Cross-Section mass (per unit length)
            obj.m = baseParameters.rho * baseParameters.A;

            % Cross-Section inertia tensor
            % [LLA11, p.292]
            obj.J = baseParameters.rho * diag([baseParameters.I_x, baseParameters.I_y, baseParameters.J_P]);

            % Stiffness matrix
            obj.Cgen = diag([ ...
                baseParameters.E * baseParameters.I_x, ...
                baseParameters.E * baseParameters.I_y, ...
                baseParameters.G * baseParameters.J_P, ...
                baseParameters.G * baseParameters.A, ...
                baseParameters.G * baseParameters.A, ...
                baseParameters.E * baseParameters.A]);

            % Generalized cross-section mass matrix
            obj.Mgen    = blkdiag(obj.J, obj.m * eye(3));
            obj.MgenInv = inv(obj.Mgen);
        end
    end
end
