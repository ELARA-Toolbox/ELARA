classdef MBBeamParams
    %% BeamParams Class storing all parameters defining a geometrically exact beam
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Beam Geometry
        geom (1,1)  beamCrossSecGeomParams

        % Beam Material
        mat (1,1)   beamMaterialParams

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
        function obj = computeParams(obj)
            %% Compute missing parameters
            % from the known input parameters

            assert(~isempty(obj.mat.E));
            assert(~isempty(obj.mat.nu));
            assert(~isempty(obj.mat.rho));
            assert(~isempty(obj.geom.A));
            assert(~isempty(obj.geom.I_x));
            assert(~isempty(obj.geom.I_y));
            assert(~isempty(obj.geom.J_P))

            % Shear Modulus
            % [LLA11, p.307], also e.g., [Dem+15, p. 80]
            obj.mat.G = obj.mat.E / ( 2 * (1 + obj.mat.nu ) );

            % Cross-Section mass (per unit length)
            obj.m = obj.mat.rho * obj.geom.A;

            % Cross-Section inertia tensor
            % [LLA11, p.292]
            obj.J = obj.mat.rho * diag([obj.geom.I_x, obj.geom.I_y, obj.geom.J_P]);

            % Stiffness matrix
            obj.Cgen = diag([ ...
                obj.mat.E * obj.geom.I_x, ...
                obj.mat.E * obj.geom.I_y, ...
                obj.mat.G * obj.geom.J_P, ...
                obj.mat.G * obj.geom.A, ...
                obj.mat.G * obj.geom.A, ...
                obj.mat.E * obj.geom.A]);

            % Generalized cross-section mass matrix
            obj.Mgen    = blkdiag(obj.J, obj.m * eye(3));
            obj.MgenInv = inv(obj.Mgen);
        end
    end
end
