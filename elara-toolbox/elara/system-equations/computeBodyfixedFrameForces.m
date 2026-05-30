function f_frame_b = computeBodyfixedFrameForces(g, f_frame_s, MBSys, simPars)
    %% Compute the body-fixed forces for all frames: Gravity and Ext. Forces
    %
    f_frame_b = zeros(6,MBSys.nFrames);
    for iFrm = 1:MBSys.nFrames
        f_frame_b(:, iFrm) = ...
            ... % External spatial forces
            + [
            g(1:3,1:3,iFrm).' * -f_frame_s(1:3,iFrm)
            g(1:3,1:3,iFrm).' * -f_frame_s(4:6,iFrm)
            ] ...
            ...% Gravity
            + simPars.g * [
            MBSys.frameData.m_a(iFrm) * cross( MBSys.frameData.x_a(:, iFrm), g(1:3,1:3,iFrm).' * [0;0;1] )
            MBSys.frameData.m(iFrm) * g(1:3,1:3,iFrm).' * [0;0;1] 
            ];
    end
end