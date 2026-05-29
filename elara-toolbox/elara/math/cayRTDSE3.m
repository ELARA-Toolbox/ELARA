function T = cayRTDSE3( xi )
    %% Right-Trivialized Derivative of the Cayley map for SE(3)
    % Implements the right-trivialized derivative for the Cayley map (also
    % referred to as the right-trivialized tangent map) for SE(3):
    % dcay : se(3) -> se(3) as the corresponding 6x6 linear matrix operator
    %
    % Source: [Kob14], eq. 17; cf. [Dem+14, p.11];
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
    om = xi(1:3);
    v  = xi(4:end);

    omH = skew(om);
    vH  = skew(v);
    
    T = zeros(6, 6, class(xi));
    T(1:3, 1:3) = 2 / (4 + om.'*om) * ( 2*eye(3) + omH );
    T(1:3, 4:6) = zeros(3);
    T(4:6, 1:3) = 1 / (4 + om.'*om) * vH * ( 2*eye(3) + omH );
    T(4:6, 4:6) = eye(3) + ( 1 / (4 + om.'*om) * ( 2*omH + omH^2 ) );

    %}


    %% More efficient code generated from MATLAB
    % (using the above initial implementation)
    % Code used:
    % xi = sym('xi', [6,1]);
    % T = cayRTDSE3( xi );
    % matlabFunction(T, "Vars", {xi}, "File","tempFun", "Optimize",true);
    
    xi1 = xi(1,:);
    xi2 = xi(2,:);
    xi3 = xi(3,:);
    xi4 = xi(4,:);
    xi5 = xi(5,:);
    xi6 = xi(6,:);
    t2 = xi1.*xi2;
    t3 = xi1.*xi3;
    t4 = xi2.*xi3;
    t5 = xi1.*2.0;
    t6 = xi2.*2.0;
    t7 = xi3.*2.0;
    t8 = xi1.^2;
    t9 = xi2.^2;
    t10 = xi3.^2;
    t11 = t8+t9+t10+4.0;
    t12 = 1.0./t11;
    t13 = t12.*4.0;
    t14 = t5.*t12;
    t15 = t6.*t12;
    t16 = t7.*t12;
    t17 = t12.*xi4.*2.0;
    t18 = t12.*xi5.*2.0;
    t19 = t12.*xi6.*2.0;
    t20 = t12.*xi1.*xi4;
    t21 = t12.*xi2.*xi5;
    t22 = t12.*xi3.*xi6;
    t23 = -t20;
    t24 = -t21;
    t25 = -t22;
    T = reshape([t13,t16,t12.*xi2.*-2.0,t24+t25,t19+t12.*xi2.*xi4,-t18+t12.*xi3.*xi4,t12.*xi3.*-2.0,t13,t14,-t19+t12.*xi1.*xi5,t23+t25,t17+t12.*xi3.*xi5,t15,t12.*xi1.*-2.0,t13,t18+t12.*xi1.*xi6,-t17+t12.*xi2.*xi6,t23+t24,0.0,0.0,0.0,-t12.*(t9+t10)+1.0,t12.*(t2+t7),t12.*(t3-t6),0.0,0.0,0.0,t12.*(t2-t7),-t12.*(t8+t10)+1.0,t12.*(t4+t5),0.0,0.0,0.0,t12.*(t3+t6),t12.*(t4-t5),-t12.*(t8+t9)+1.0],[6,6]);

end

