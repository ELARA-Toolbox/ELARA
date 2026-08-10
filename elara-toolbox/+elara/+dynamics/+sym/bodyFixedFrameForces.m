function f_frame_b = bodyFixedFrameForces(system, g, f_frame_s, g_grav)
    %% Compute gravity and external forces in body-fixed coordinates for all frames
    arguments
        system      (1,1) elara.SystemSym

        % Absolute configurations of all frames
        g           (1,:) elara.SE3.Element

        % Spatial forces (twists) of all frames
        f_frame_s   (6,:)

        % Gravity constant
        g_grav      (1,1)
    end

    f_frame_b = cell(1, system.nFrames);

    for iFrm = 1:system.nFrames
        f_frame_b{iFrm} = ...
            ... % External spatial forces
            [
            g(iFrm).R.' * -f_frame_s(1:3,iFrm)
            g(iFrm).R.' * -f_frame_s(4:6,iFrm)
            ] ...
            ...% Gravity
            + g_grav * [
            system.frames.m_a(iFrm) * cross( system.frames.x_a(:,iFrm), g(iFrm).R.' * [0;0;1] )
            system.frames.m(iFrm) * g(iFrm).R.' * [0;0;1]
            ];
    end
end
