function links = systemDef_cantilever_system
    %% Define MBS System: Rigid-flexible Multibody System

    %% Link 1: Flexible cantilever link
    links(1) = elara.FlexibleLink;

    % Define as a cantilever beam that is fixed to the base
    links(1).parentLink = 0;
    links(1).isCantilever = true;

    % Discretize with 5 segments
    links(1).nSegments      = 5;

    % Specify beam length
    links(1).L         = 0.5;

    % Transformation from the joint frame to the first beam node (unit
    % transformation)
    links(1).g_J_B     = eye(4);

    % Model as inextensible Kirchhoff beam: Only include the two bending
    % and the torsion deformation modes
    links(1).Ba = [ eye(3); zeros(3)];
    links(1).Bc = [ zeros(3); eye(3)];

    % Specify straight reference configuration
    links(1).xiRef = repmat([0;0;0;0;0;1], [1,links(1).nSegments]);

    % Define beam material and geometry parameters
    links(1).beamParameters = beamParams_ASA_round("radius",0.006);

    % Define material/Kelvin-Voigt dissipation
    links(1).beamParameters.d = ones(6,1)*1e-3;


    %% Link 2: Rigid link
    links(2) = elara.RigidLink;

    % Link is attached to the cantilever beam (link #1)
    links(2).parentLink = 1;

    % Define joint kinematics and properties
    links(2).jointIsActuated = 1;
    links(2).jointAxis  = [0 1 0 0 0 0].';
    links(2).g_J_B      = elara.SE3.matrix(eye(3), [0,0,0.3]);
    links(2).g_ref      = elara.SE3.matrix(eye(3), [0,0,0.3]);

    % Link mass and inertia
    links(2).m          = 0.5;
    links(2).J          = diag([1,1,1e-3])*1e-4;

    % Add bounding box centered at COM
    links(2).g_bbox = eye(4);
    links(2).bBoxSize = [
        +0.08, +0.02, +0.3
        -0.08, -0.02, -0.3
        ];

    %% Link 3: Rigid link

    % Transformation link 2 COM -> link 3 joint
    g_COM2_J3 = elara.SE3.matrix(eye(3), [0,0,0.3]);

    % Transformation link 3 joint -> link 3 COM
    g_J3_COM3 = elara.SE3.matrix(eye(3), [0,0,0.3]);

    links(3) = elara.RigidLink;

    links(3).parentLink = 2;
    links(3).jointIsActuated = 1;
    links(3).jointAxis  = [0 1 0 0 0 0].';
    links(3).g_J_B      = g_J3_COM3;
    links(3).g_ref      = g_COM2_J3*g_J3_COM3;
    links(3).m          = 0.3;
    links(3).J          = diag([1,1,1e-3])*5e-5;

    % Add bounding box centered at COM
    links(3).g_bbox = eye(4);
    links(3).bBoxSize = [
        +0.08, +0.02, +0.3
        -0.08, -0.02, -0.3
        ];

    % Define joint dissipation for all links
    links(1).d = 1e-1; % Unused, since the first link (the beam) is fixed
    links(2).d = 1e-1;
    links(3).d = 1e-1;
end