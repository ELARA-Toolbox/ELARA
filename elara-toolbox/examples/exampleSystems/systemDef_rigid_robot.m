function links = systemDef_rigid_robot(opts)
    %% Define MBS System: Rigid three-link lab robot
    arguments
        opts.d (1,1) double = 1e-2;
        opts.l (3,1) double = [0.2,0.5,0.5];
    end

    nLinks = 3;
    links = createArray(nLinks,1,"MBLinkDefinitionRigid");

    % Joint dissipation
    links(1).d = opts.d;
    links(2).d = opts.d;
    links(3).d = opts.d;

    %% Define link geometry

    % Link lengths
    % l(1) = 0.2;
    % l(2) = 0.5;
    % l(3) = 0.5;
    l = opts.l;

    % Constant transformations in the joints
    % (between adjacent joint frames)
    g_J_const(:,:,1) = eye(4);
    g_J_const(:,:,2) = SE3Matrix([0, -1, 0; 0 0 -1; 1, 0, 0], zeros(3,1));
    g_J_const(:,:,3) = eye(4);

    % Transformations from joint 1 -> joint 2 in each (serial) link
    g_J1_J2(:,:,1)  = SE3Matrix(eye(3), [0,0,l(1)]);
    g_J1_J2(:,:,2)  = SE3Matrix(eye(3), [l(2),0,0]);
    g_J1_J2(:,:,3)  = SE3Matrix(eye(3), [l(3),0,0]);

    % Transformations from joint 1 -> COM in each (serial) link
    g_J1_COM(:,:,1) = SE3Matrix(eye(3), [0,0,l(1)/2]);
    g_J1_COM(:,:,2) = SE3Matrix(eye(3), [l(2)/2,0,0]);
    g_J1_COM(:,:,3) = SE3Matrix(eye(3), [l(3)/2,0,0]);

    % Transformations from COM -> joint 2
    g_COM_J2 = zeros(4,4,nLinks);
    for iLink = 1:nLinks
        g_COM_J2(:,:,iLink) = g_J1_COM(:,:,iLink) \ g_J1_J2(:,:,iLink);
    end

    % Check SE3 arrays
    mustBeSE3MatrixArray(g_J_const);
    mustBeSE3MatrixArray(g_J1_COM);
    mustBeSE3MatrixArray(g_COM_J2);


    %% Define joints

    % Screw vectors in the joint frames
    Z(:,1) = [0 0 1 0 0 0].';
    Z(:,2) = [0 0 1 0 0 0].';
    Z(:,3) = [0 0 1 0 0 0].';

    g_J_ref = zeros(4,4,nLinks);
    % Reference transformations of the joints
    for iLink = 1:nLinks
        if iLink == 1
            % First body in the chain: Only transformation from
            % joint attachement
            g_J_ref(:,:,iLink) = g_J_const(:,:,iLink) * g_J1_COM(:,:,iLink);
        else
            g_J_ref(:,:,iLink) = g_COM_J2(:,:,iLink-1) * g_J_const(:,:,iLink) * g_J1_COM(:,:,iLink);
        end
    end
    mustBeSE3MatrixArray(g_J_ref);


    %% Create link objects

    %%% Link 1: Base link
    rho = 2700; % Density
    r = 0.050;  % radius
    m = r^2 * pi * rho * l(1); % mass
    I_sym  = 1/2*m*r^2;
    I_long = 1/4*m*r^2 + 1/12*m*l(1)^2;

    links(1).parentLink = 0;
    links(1).isActuated = 1;
    links(1).jointAxis  = Z(:,1);
    links(1).g_J_B      = g_J1_COM(:,:,1);
    links(1).g_ref      = g_J_ref(:,:,1);
    links(1).m          = m;
    links(1).J          = diag([I_long, I_long, I_sym]);

    % Add bounding box centered at COM
    links(1).g_bbox = eye(4);
    links(1).bBoxSize = [
        +0.08, +0.08, +l(1)/2
        -0.08, -0.08, -l(1)/2
        ];

    %%% Link 2: First arm, standard rigid arm

    r = 0.030; % radius
    m = r^2 * pi * rho * l(2); % mass
    I_sym  = 1/2*m*r^2;
    I_long = 1/4*m*r^2 + 1/12*m*l(2)^2;

    links(2).parentLink = 1;
    links(2).jointAxis  = Z(:,2);
    links(2).g_J_B      = g_J1_COM(:,:,2);
    links(2).g_ref      = g_J_ref(:,:,2);
    links(2).m          = m;
    links(2).J          = diag([I_sym, I_long, I_long]);

    % Add bounding box centered at COM
    links(2).g_bbox = eye(4);
    links(2).bBoxSize = [
        +l(2)/2+0.05, +0.06, +0.00
        -l(2)/2-0.05, -0.06, -0.06
        ];

    %%% Link 3: Rigid arm
    r = 0.030; % radius
    m = r^2 * pi * rho * l(3); % mass
    I_sym  = 1/2*m*r^2;
    I_long = 1/4*m*r^2 + 1/12*m*l(3)^2;

    links(3).parentLink = 2;
    links(3).jointAxis  = Z(:,3);
    links(3).g_J_B      = g_J1_COM(:,:,3);
    links(3).g_ref      = g_J_ref(:,:,3);
    links(3).m          = m;
    links(3).J          = diag([I_sym, I_long, I_long]);

    % Add bounding box centered at COM
    links(3).g_bbox = eye(4);
    links(3).bBoxSize = [
        +l(3)/2, +0.05, +0.05
        -l(3)/2-0.05, -0.05, -0.00
        ];

    % Add TCP definition
    links(3).hasTCP = true;
    links(3).g_B_TCP = SE3Matrix(eye(3), [l(3)/2;0;0]);

end