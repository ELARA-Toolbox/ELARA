function T = cayRTDInvSE3( xi )
    %% Inverse Right-Trivialized Derivative of the Cayley map for SE(3)
    % Implements the inverse of the right-trivialized derivative for the
    % Cayley map (also referred to as the right-trivialized tangent map)
    % for SE(3): dcay : se(3) -> se(3) as the corresponding 6x6 linear
    % matrix operator
    %
    % Source: [KM11], eq. 34, [Dem+14] eq. 20
    % cf. [KM11] and other sources for background information
    %
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Output T: 6x6 matrix
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        xi (6,1)
    end

    %% Original implementation using "analytic" functions
    %{
    omH = skewSO3( xi(1:3) );
    vH  = skewSO3( xi(4:6) );

    T = zeros(6, 6, class(xi));
    T(1:3, 1:3) = eye(3) - (1/2 * omH) + (1/4 * (xi(1:3) * xi(1:3).') );
    T(1:3, 4:6) = zeros(3);
    T(4:6, 1:3) = -1/2 * (eye(3) - 1/2 * omH ) * vH;
    T(4:6, 4:6) = eye(3) -1/2 * omH;
    %}

    %% More efficient code generated from MATLAB
    % (using the above initial implementation)
    % Code used:
    % xi = sym('xi', [6,1]);
    % CR = cayRTDInvSE3( xi );
    % matlabFunction(CR, "Vars", {xi}, "File","tempFun", "Optimize",true);

    xi1 = xi(1,:);
    xi2 = xi(2,:);
    xi3 = xi(3,:);
    xi4 = xi(4,:);
    xi5 = xi(5,:);
    xi6 = xi(6,:);
    t2 = xi1./2.0;
    t3 = xi2./2.0;
    t4 = xi3./2.0;
    t5 = xi4./2.0;
    t6 = xi5./2.0;
    t7 = xi6./2.0;
    t8 = (xi1.*xi2)./4.0;
    t9 = (xi1.*xi3)./4.0;
    t10 = (xi1.*xi4)./4.0;
    t11 = (xi2.*xi3)./4.0;
    t12 = (xi2.*xi5)./4.0;
    t13 = (xi3.*xi6)./4.0;
    t14 = -t2;
    t15 = -t3;
    t16 = -t4;
    t17 = -t10;
    t18 = -t12;
    t19 = -t13;
    T = reshape([xi1.^2./4.0+1.0,t8+t16,t3+t9,t18+t19,-t7+(xi1.*xi5)./4.0,t6+(xi1.*xi6)./4.0,t4+t8,xi2.^2./4.0+1.0,t11+t14,t7+(xi2.*xi4)./4.0,t17+t19,-t5+(xi2.*xi6)./4.0,t9+t15,t2+t11,xi3.^2./4.0+1.0,-t6+(xi3.*xi4)./4.0,t5+(xi3.*xi5)./4.0,t17+t18,0.0,0.0,0.0,1.0,t16,t3,0.0,0.0,0.0,t4,1.0,t14,0.0,0.0,0.0,t15,t2,1.0],[6,6]);

end
