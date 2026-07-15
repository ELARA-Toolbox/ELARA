function f_frame_b = computeBodyfixedFrameForces_sym(MBSys, g, f_frame_s, g_grav)
    %% Compute the body-fixed forces for all frames: Gravity and Ext. Forces
    arguments
        MBSys       (1,1) elara.SystemSym

        % Absolute configurations of all frames
        g           (1,:) SE3

        % Spatial forces (twists) of all frames
        f_frame_s   (6,:)

        % Gravity constant
        g_grav      (1,1)
    end

    f_frame_b = cell(1, MBSys.nFrames);

    for iFrm = 1:MBSys.nFrames
        f_frame_b{iFrm} = ...
            ... % External spatial forces
            [
            g(iFrm).R.' * -f_frame_s(1:3,iFrm)
            g(iFrm).R.' * -f_frame_s(4:6,iFrm)
            ] ...
            ...% Gravity
            + g_grav * [
            MBSys.frameData.m_a(iFrm) * cross( MBSys.frameData.x_a(:,iFrm), g(iFrm).R.' * [0;0;1] )
            MBSys.frameData.m(iFrm) * g(iFrm).R.' * [0;0;1]
            ];
    end
end