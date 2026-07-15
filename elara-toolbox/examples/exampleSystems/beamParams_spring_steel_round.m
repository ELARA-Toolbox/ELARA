function pars = beamParams_spring_steel_round(opts)
    %% Beam parameters for spring steel rod with circular cross-section
    % As identified in Semesterarbeit Tobias Farger (WS23/24, #563)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    %
    arguments
        % Radius
        opts.radius (1,1) double = 2e-3;

        % Factor for Young's Modulus
        % (as a straightforward tuning method)
        opts.factorE (1,1) double = 1;
    end

    % Class Instance
    pars = elara.BeamParams;

    % Struct with intermediate data
    baseParams = struct();

    %% Beam Geometry

    % Cross-Section geometry
    % H/W corresponds to the diameter of the circular cross-section
    radius = opts.radius;
    pars.H = 2*radius;
    pars.W = 2*radius;
    baseParams.A = radius^2 * pi;

    % Compute second moments of inertia (about x and y axes of the body-fixed
    % coordinate systems)
    % https://en.wikipedia.org/wiki/List_of_second_moments_of_area
    baseParams.I_x = pi/4 * radius^4;
    baseParams.I_y = pi/4 * radius^4;

    % Polar moment of inertia
    baseParams.J_P = pi/2 * radius^4;

    %% Beam Material
    % Density (kg/m^3)
    % From HiWi documentation, table 2.3
    baseParams.rho = 7.9e3; % Beispiel literatur
    % 8.211857e3; % Hiwi doc

    % Young's modulus (N/m^2)
    % Average value from term paper, p. 28
    %pars.mat.E = 207.8695e9 * opts.factorE; Hiwi doc
    baseParams.E = 185e9 * opts.factorE; % online

    % Poisson's number
    % Average value from term paper, p.29
    %pars.mat.nu = 0.3660;
    baseParams.nu = 0.3; % online

    %% Dissipation coefficients
    pars.d = 0;

    % Compute Generalized Stiffness and Inertia matrices
    pars = pars.computeParams(baseParams);
    
end
