function Ad = lAdSE3(g)
    % Large Ad representation (6x6 matrix) of an element of SE3
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input g: SE3 element in (4x4) matrix representation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        g (4,4)
    end

    %% Original implementation using "analytic" functions
    %{
    R = g(1:3, 1:3);
    p = g(1:3, 4);

    % Ad representation, taken from [Par+18], eq. (40)
    Ad = [
        R,         zeros(3);
        skew(p)*R, R;
        ];

    %}


    %% More efficient code generated from MATLAB
    % (using the above initial implementation)
    % Code used:
    % g = sym('g', [4,4]);
    % Ad = lAdSE3(g);
    % matlabFunction(Ad, "Vars", {g}, "File","tempFun", "Optimize",true);

    g1_1 = g(1);
    g1_2 = g(5);
    g1_3 = g(9);
    g1_4 = g(13);
    g2_1 = g(2);
    g2_2 = g(6);
    g2_3 = g(10);
    g2_4 = g(14);
    g3_1 = g(3);
    g3_2 = g(7);
    g3_3 = g(11);
    g3_4 = g(15);
    Ad = reshape([g1_1,g2_1,g3_1,-g2_1.*g3_4+g2_4.*g3_1,g1_1.*g3_4-g1_4.*g3_1,-g1_1.*g2_4+g1_4.*g2_1,g1_2,g2_2,g3_2,-g2_2.*g3_4+g2_4.*g3_2,g1_2.*g3_4-g1_4.*g3_2,-g1_2.*g2_4+g1_4.*g2_2,g1_3,g2_3,g3_3,-g2_3.*g3_4+g2_4.*g3_3,g1_3.*g3_4-g1_4.*g3_3,-g1_3.*g2_4+g1_4.*g2_3,0.0,0.0,0.0,g1_1,g2_1,g3_1,0.0,0.0,0.0,g1_2,g2_2,g3_2,0.0,0.0,0.0,g1_3,g2_3,g3_3],[6,6]);

end