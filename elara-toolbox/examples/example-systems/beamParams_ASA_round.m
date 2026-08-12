function pars = beamParams_ASA_round(options)
    %% Beam parameters for ASA Rod with Circular Cross-Section
    % As identified in the semester thesis from Tobias Farger (WS23/24, #563)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    %
    arguments
        % Individual Value for the beam length (m)
        options.L (1,1) double = 1;

        % Radius
        options.radius (1,1) double = 3e-3;

        options.E (1,1) double = 2279.5e6;
    end

    % Class Instance
    pars = elara.BeamParameters;

    % Struct with intermediate data
    baseParams = struct();

    %% Beam Parameters

    %%% Beam Geometry
    % with circular cross-section
    radius = options.radius;

    % Cross-Section geometry
    % H/W corresponds to the diameter of the circular cross-section
    pars.height = 2*radius;
    pars.width = 2*radius;
    baseParams.A = radius^2 * pi;

    % Compute second moments of inertia (about x and y axes of the body-fixed
    % coordinate systems)
    % https://en.wikipedia.org/wiki/List_of_second_moments_of_area
    baseParams.I_x = pi/4 * radius^4;
    baseParams.I_y = pi/4 * radius^4;

    % Polar moment of inertia
    baseParams.J_P = pi/2 * radius^4;

    %%% Beam Material
    % Density (kg/m^3)
    baseParams.rho = 1070;

    % Young's modulus (N/m^2)
    baseParams.E = options.E;

    % Poisson's number
    baseParams.nu = 0.3459;


    %%% Dissipation coefficients
    pars.d = 0;

    % Compute Generalized Stiffness and Inertia matrices
    pars = pars.computeParameters(baseParams);
   
end
