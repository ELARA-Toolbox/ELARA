function links = systemDef_cantilever_system
    %% Define MBS System: Rigid-flexible MB system

    %% Link 1: Cantilever link

    links(1) = elara.FlexibleLink;

    links(1).parentLink = 0;
    links(1).isCantilever = true;
    links(1).nSegments      = 5;
    links(1).L         = 0.5;
    links(1).g_J_B     = eye(4);
    links(1).Ba = [ eye(3); zeros(3)];
    links(1).Bc = [ zeros(3); eye(3)];
    links(1).xiRef = repmat([0;0;0;0;0;1], [1,links(1).nSegments]);
    links(1).beamParameters = beamParams_ASA_round("radius",0.006);
    links(1).beamParameters.d = ones(6,1)*1e-3;


    %% Link 2: Rigid link

    links(2) = elara.RigidLink;

    links(2).parentLink = 1;
    links(2).jointIsActuated = 1;
    links(2).jointAxis  = [0 1 0 0 0 0].';
    links(2).g_J_B      = SE3Matrix(eye(3), [0,0,0.3]);
    links(2).g_ref      = SE3Matrix(eye(3), [0,0,0.3]);
    links(2).m          = 0.5;
    links(2).J          = diag([1,1,1e-3])*1e-4;

    % Add bounding box centered at COM
    links(2).g_bbox = eye(4);
    links(2).bBoxSize = [
        +0.08, +0.02, +0.3
        -0.08, -0.02, -0.3
        ];

    %% Link 3: Rigid link

    % Transf. link 2 COM -> link 3 joint
    g_COM2_J3 = SE3Matrix(eye(3), [0,0,0.3]);

    % Transf. link 3 joint -> link 3 COM
    g_J3_COM3 = SE3Matrix(eye(3), [0,0,0.3]);

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

    links(1).d = 1e-1;
    links(2).d = 1e-1;
    links(3).d = 1e-1;
end