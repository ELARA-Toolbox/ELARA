function links = systemDef_planarNLinkPendulum(opts)
    %% Define MBS System: Planar two-link manipulator
    % similar to Sec. 5.2.2 in [Obe08] (2-link case), and
    % J. Brüdigam, S. Sosnowski, Z. Manchester, and S. Hirche,
    % "Variational integrators and graph-based solvers for multibody
    % dynamics in maximal coordinates," Multibody Syst Dyn, vol. 61,
    % no. 3, pp. 381–414, 2024, doi: 10.1007/s11044-023-09949-x.
    arguments
        opts.d      (1,1) double = 0;
        opts.nLinks (1,1) uint8 = 4;
    end

    nLinks = opts.nLinks;

    %% Define link geometry

    % Link lengths
    l = 1;

    % Transformations from joint 1 -> joint 2 in each (serial) link
    g_J1_J2  = elara.SE3.matrix(eye(3), [l,0,0]);

    % Transformations from joint 1 -> COM in each (serial) link
    g_J1_COM = elara.SE3.matrix(eye(3), [l/2,0,0]);

    % Transformations from COM -> joint 2
    g_COM_J2 = g_J1_COM \ g_J1_J2;

    % Joint transformations
    % First body in the chain: Only transformation from joint attachment
    g_J_ref_1 = g_J1_COM;

    % Other bodies: Include transformation from previous link
    g_J_ref_i = g_COM_J2 * g_J1_COM;


    %% Inertia properties
    rho = 2700; % Density
    m = 1;      % unit mass
    r = sqrt(m / (rho * pi *l)); % Radius corresponding to density and mass

    % Inertia tensor: Inertia from rigid cylinders
    I_sym  = 1/2*m*r^2;
    I_long = 1/4*m*r^2 + 1/12*m*l^2;
    J = diag([I_sym, I_long, I_long]);

    %% Create link objects

    links = createArray(nLinks, 1, "elara.RigidLink");

    for iLink = 1:nLinks
        if iLink == 1
            links(iLink).parentLink = 0;
            links(iLink).g_ref      = g_J_ref_1;
        else
            links(iLink).parentLink = iLink - 1;
            links(iLink).g_ref      = g_J_ref_i;
        end

        links(iLink).jointIsActuated = 1;
        links(iLink).jointAxis  = [0 1 0 0 0 0];
        links(iLink).g_J_B      = g_J1_COM;
        links(iLink).m          = m;
        links(iLink).J          = J;
        links(iLink).d          = opts.d;

        % Add bounding box centered at COM
        links(iLink).g_bbox = eye(4);
        links(iLink).bBoxSize = [
            +l/2+0.05, +0.02, +0.06
            -l/2-0.05, -0.02, -0.06
            ];
    end
end