function AdInv = lAdSE3Inv(g)
    % *Inverse* large Ad representation (6x6 matrix) of an element of SE3
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

    % Inverse Ad representation, taken from [MLS94, p. 56]
    AdInv = [
        R.',          zeros(3);
        -R.'*skew(p), R.';
        ];
    %}


    %% More efficient code generated from MATLAB
    % (using the above initial implementation)
    % Code used:
    % g = sym('g', [4,4]);
    % AdInv = lAdSE3Inv( g );
    % matlabFunction(AdInv, "Vars", {g}, "File","tempFun", "Optimize",true);

    AdInv = reshape([g(1),g(5),g(9),-g(2).*g(15)+g(14).*g(3),-g(6).*g(15)+g(14).*g(7),-g(10).*g(15)+g(14).*g(11),g(2),g(6),g(10),g(1).*g(15)-g(13).*g(3),g(5).*g(15)-g(13).*g(7),g(9).*g(15)-g(13).*g(11),g(3),g(7),g(11),-g(1).*g(14)+g(13).*g(2),-g(5).*g(14)+g(13).*g(6),-g(9).*g(14)+g(13).*g(10),0.0,0.0,0.0,g(1),g(5),g(9),0.0,0.0,0.0,g(2),g(6),g(10),0.0,0.0,0.0,g(3),g(7),g(11)],[6,6]);

end