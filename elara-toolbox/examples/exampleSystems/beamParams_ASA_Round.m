function pars = beamParams_ASA_Round(options)
    %% Beam parameters for ASA Rod with Circular Cross-Section
    % As identified in Semesterarbeit Tobias Farger (WS23/24, #563)
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
    pars = MBBeamParams;

    %% Beam Parameters

    %%% Beam Geometry
    % with circular cross-section
    radius = options.radius;

    % Cross-Section geometry
    % H/W corresponds to the diameter of the circular cross-section
    pars.geom.H = 2*radius;
    pars.geom.W = 2*radius;
    pars.geom.A = radius^2 * pi;

    % Compute second moments of inertia (about x and y axes of the body-fixed
    % coordinate systems)
    % https://en.wikipedia.org/wiki/List_of_second_moments_of_area
    pars.geom.I_x = pi/4 * radius^4;
    pars.geom.I_y = pi/4 * radius^4;

    % Polar moment of inertia
    pars.geom.J_P = pi/2 * radius^4;

    %%% Beam Material
    % Density (kg/m^3)
    pars.mat.rho = 1070;

    % Young's modulus (N/m^2)
    pars.mat.E = options.E;

    % Poisson's number
    pars.mat.nu = 0.3459;


    %%% Dissipation coefficients
    pars.d = 0;

    pars = pars.computeParams;
   
end
