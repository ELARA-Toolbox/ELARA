function links = systemDef_rigid_flexible_robot(opts)
    %% Define MBS System: "Surgical" Rigid-Soft Manipulator
    % With three rigid base links and a flexible instrument as a third
    % link; inspired by systems like the DLR Miro
    arguments
        opts.dJoints    (1,1) double = 1e-2;
        opts.dBeam      (6,1) double = ones(6,1)*5e-4;
        opts.nSeg       (1,1) uint8  = 6;
    end

    % Rigid base links
    lR = [0.25,0.4,0.4];
    linksR = systemDef_rigid_robot("d", opts.dJoints, "l", lR);

    linksR(end).hasTCP = false;

    %% Flexible link

    linkF = MBLinkDefinitionFlexible;

    linkF.parentLink   = 3;
    linkF.isCantilever = false;
    linkF.isActuated   = true;
    linkF.nSeg         = opts.nSeg;
    linkF.L            = 0.4;
    linkF.g_J_B        = eye(4);
    linkF.Ba           = [ eye(3); zeros(3)];
    linkF.Bc           = [ zeros(3); eye(3)];
    linkF.Ba           = [ eye(2); zeros(4,2)];
    linkF.Bc           = [ zeros(2,4); eye(4)];
    linkF.xiRef        = repmat([0;0;0;0;0;1], [1,linkF.nSeg]);
    linkF.beamPars   = beamParams_ASA_round("radius", 2.75e-3);
    linkF.beamPars.d = opts.dBeam;
    linkF.jointAxis  = [1 0 0 0 0 0].';
    linkF.g_ref = SE3Matrix([0, 0, 1; 0 1 0; -1, 0, 0], [lR(end)/2+0.05,0,0]);
    linkF.g_J_B = SE3Matrix(eye(3), [0;0;0.1]);
    linkF.d     = opts.dJoints;

    % External masses for the beam: Mass for the joint and an end effector
    % Implemented with simple steel balls
    % (assumption: Body CoM frame = CS frame)

    rBall_1 = 3e-2; % Mass at the joint
    rBall_2 = 0.7e-2; % Mass of the end effector

    % Mass of the rigid body
    rho = 8.211e3;
    m_b_1 = rho * rBall_1^3 * pi * 4/3;
    m_b_2 = rho * rBall_2^3 * pi * 4/3;

    % Inertia tensor of the rigid body (w.r.t. body CoM frame = CS frame)
    J_b_1 = eye(3) * m_b_1 * 2/5 * rBall_1^2;
    J_b_2 = eye(3) * m_b_2 * 2/5 * rBall_2^2;

    % Add to link definition
    linkF.g_a = repmat(eye(4), [1,1,linkF.nSeg+1]); % All masses located in node (CS) frames
    linkF.m_a = zeros(linkF.nSeg+1,1);
    linkF.M_a = zeros(6,6,linkF.nSeg+1);

    linkF.m_a(1)       = m_b_1;
    linkF.m_a(end)     = m_b_2;
    linkF.M_a(:,:,1)   = blkdiag(J_b_1, eye(3)*m_b_1);
    linkF.M_a(:,:,end) = blkdiag(J_b_2, eye(3)*m_b_2);

    % Add TCP definition
    linkF.hasTCP = true;
    linkF.g_B_TCP = SE3Matrix(eye(3), [0,0,0]);


    %% Assemble system links
    links = [linksR;linkF];

end