classdef BeamParameters
    %% BeamParameters Class storing all parameters defining a geometrically exact beam
    %
    % Maximilian heighterrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        %%% Beam Geometry
        height  (1,1) double {mustBePositive,mustBeFinite} = 1e-2;
        width   (1,1) double {mustBePositive,mustBeFinite} = 1e-2;

        %%% Mixed parameters
        % Cross-Section mass (per unit length)
        m       (1,1) double {mustBePositive,mustBeFinite} = 1;

        % Cross-Section inertia tensor
        J       (3,3) double {mustBeFinite} = eye(3);

        % Stiffness matrix
        Cgen    (6,6) double {mustBeFinite} = eye(6); 

        % Generalized cross-section mass matrix
        Mgen    (6,6) double {mustBeFinite} = eye(6);
        MgenInv (6,6) double {mustBeFinite} = eye(6);

        % Linear strain rate damping coefficient (for all segments)
        % for Kelvin-Voigt type damping
        d       (6,1) double {mustBeNonnegative,mustBeFinite} = zeros(6,1);
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
            assert(isfield(baseParameters, "E"), 'elara:MissingBeamParameter', ...
                'Missing Young''s modulus E (N/m^2).');
            assert(isfield(baseParameters, "nu"), 'elara:MissingBeamParameter', ...
                'Missing Poisson''s ratio nu.');
            assert(isfield(baseParameters, "rho"), 'elara:MissingBeamParameter', ...
                'Missing density rho (kg/m^3).');
            assert(isfield(baseParameters, "A"), 'elara:MissingBeamParameter', ...
                'Missing cross-section area A (m^2).');
            assert(isfield(baseParameters, "I_x"), 'elara:MissingBeamParameter', ...
                'Missing second moment of area I_x about the body-fixed x axis.');
            assert(isfield(baseParameters, "I_y"), 'elara:MissingBeamParameter', ...
                'Missing second moment of area I_y about the body-fixed y axis.');
            assert(isfield(baseParameters, "J_P"), 'elara:MissingBeamParameter', ...
                'Missing polar second moment of area J_P.');

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
