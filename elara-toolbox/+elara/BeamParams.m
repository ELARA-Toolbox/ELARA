classdef BeamParams
    %% BeamParams Class storing all parameters defining a geometrically exact beam
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Beam Geometry
        H      (1,1) double   % Height
        W      (1,1) double   % Width

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
        function obj = computeParams(obj, baseParams)
            %% Compute Inertia and Stiffness Matrices from "Base Parameters"
            arguments
                obj         (1,1) elara.BeamParams

                % Struct with required base parameters
                % See below for required fields
                baseParams  (1,1) struct
            end

            % Assert that the struct has all required fields
            assert(isfield(baseParams, "E"));   % Young's modulus (N/m^2)
            assert(isfield(baseParams, "nu"));  % Poisson's number
            assert(isfield(baseParams, "rho")); % Density (kg/m^3)
            assert(isfield(baseParams, "A"));   % Cross-section area (m^2)
            assert(isfield(baseParams, "I_x")); % Second moments of inertia (about x axis of the body-fixed coordinate systems)
            assert(isfield(baseParams, "I_y")); % Second moments of inertia (about y axis of the body-fixed coordinate systems)
            assert(isfield(baseParams, "J_P")); % Polar moment of inertia

            % Shear Modulus
            % [LLA11, p.307], also e.g., [Dem+15, p. 80]
            baseParams.G = baseParams.E / ( 2 * (1 + baseParams.nu ) );

            % Cross-Section mass (per unit length)
            obj.m = baseParams.rho * baseParams.A;

            % Cross-Section inertia tensor
            % [LLA11, p.292]
            obj.J = baseParams.rho * diag([baseParams.I_x, baseParams.I_y, baseParams.J_P]);

            % Stiffness matrix
            obj.Cgen = diag([ ...
                baseParams.E * baseParams.I_x, ...
                baseParams.E * baseParams.I_y, ...
                baseParams.G * baseParams.J_P, ...
                baseParams.G * baseParams.A, ...
                baseParams.G * baseParams.A, ...
                baseParams.E * baseParams.A]);

            % Generalized cross-section mass matrix
            obj.Mgen    = blkdiag(obj.J, obj.m * eye(3));
            obj.MgenInv = inv(obj.Mgen);
        end
    end
end
