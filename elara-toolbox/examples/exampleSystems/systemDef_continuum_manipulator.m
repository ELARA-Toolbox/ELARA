function link = systemDef_continuum_manipulator(opts)
    %% Define MBS System: One-link continuum manipulator
    arguments
        opts.d      (6,1) double = ones(6,1)*5e-3;
        opts.nSegments   (1,1) uint8  = 5;
    end

    link =  elara.FlexibleLink;

    %% Basic Link Configuration

    link.parentLink   = 0;
    link.isCantilever = true;
    link.jointIsActuated = false;
    link.nSegments         = opts.nSegments;
    link.L            = 0.7;
    link.g_J_B        = eye(4);
    link.Ba = [ eye(3); zeros(3)];
    link.Bc = [ zeros(3); eye(3)];
    link.xiRef = repmat([0;0;0;0;0;1], [1,link.nSegments]);
    link.beamParameters   = beamParams_spring_steel_round("radius", 1e-3);
    link.beamParameters.d = opts.d;

    %% Add external masses
    % For spacer disks / end effector / payload

    % Example: Steel ball attached at the beam tip
    % (assumption: Body CoM frame = CS frame)

    rBall = 1.0e-2;

    % Mass of the rigid body
    m_b = 8.211e3 * rBall^3 * pi * 4/3;

    % Inertia tensor of the rigid body (w.r.t. body CoM frame = CS frame)
    J_b = eye(3) * m_b * 2/5 * rBall^2;

    % Inertia matrix
    Mgen_b_cs = blkdiag(J_b, eye(3)*m_b);

    % Add to simPars
    link.g_a = repmat(eye(4), [1,1,link.nSegments+1]);
    link.m_a = zeros(link.nSegments+1,1);
    link.M_a = zeros(6,6,link.nSegments+1);

    link.m_a(end)     = m_b;
    link.M_a(:,:,end) = Mgen_b_cs;


    %% Define Tendon Path Functions

    % Functions that define the cable path by returning the x,y coordinates
    % of the tendon location in the cross-section plane

    % Straight path
    % The path in the cross-section plane is defined in polar coordinates
    % by distance from backbone d (m) and angle alpha (deg)
    x_m_fun_straight = @(s,d,alpha) [d*cosd(alpha); d*sind(alpha); 0 ];

    % Helical path, taken from [RW11], Table 1
    % Parameters d and p
    x_m_fun_helical = @(s, d, p) [
        d* cos(2*pi/p*s);
        d* sin(2*pi/p*s);
        0
        ];


    %% Set Up Cable Configuration

    % Cell array of function handles; defines the individual cable paths
    link.tendonActuation.x_td_funs = {
        @(s)x_m_fun_straight(s,0.02, 0)
        @(s)x_m_fun_straight(s,0.02, 120)
        @(s)x_m_fun_straight(s,0.02, 240)
        @(s)x_m_fun_helical(s, 0.02, link.L)
        };

    % Compute derivatives of cable path functions
    link.tendonActuation = link.tendonActuation.getSymbolicPathDerivatives;

    % Lengths at which the cables terminate along the link length
    link.tendonActuation.LTermination = [
        link.L, link.L, link.L,link.L
        ];

    %% Define TCP
    link.hasTCP = true;
    link.g_B_TCP = elara.SE3.matrix(eye(3), [0,0,0.01]);
end