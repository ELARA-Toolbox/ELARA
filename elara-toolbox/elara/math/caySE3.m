function g = caySE3( xi )
    %% Cayley map for SE(3)
    % Implements the Cayley map for SE(3): cay : se(3) -> SE(3)
    %
    % Source: [Dem+14, p.10], eq. 19
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Output g: Corresponding element of SE3 in matrix form
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
    v  = xi(4:6);

    omH = skew(om);
    
    R = caySO3( om );
    x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.') ) * v;

    g = SE3Matrix(R, x);
    %}


    %% More efficient code generated from MATLAB
    % (using the above initial implementation)
    % Code used:
    % xi = sym('xi', [6,1]);
    % C = caySE3( xi );
    % matlabFunction(C, "Vars", {xi}, "File","tempFun", "Optimize",true);

    xi1 = xi(1,:);
    xi2 = xi(2,:);
    xi3 = xi(3,:);
    xi4 = xi(4,:);
    xi5 = xi(5,:);
    xi6 = xi(6,:);
    t2 = xi1.^2;
    t3 = xi2.^2;
    t4 = xi3.^2;
    t5 = xi1./2.0;
    t6 = xi2./2.0;
    t7 = xi3./2.0;
    t10 = (xi1.*xi2)./4.0;
    t12 = (xi1.*xi3)./4.0;
    t13 = (xi2.*xi3)./4.0;
    t8 = t5.*xi2;
    t9 = t5.*xi3;
    t11 = t6.*xi3;
    t14 = t2./2.0;
    t15 = t3./2.0;
    t16 = t4./2.0;
    t17 = t2+t3+t4+4.0;
    t18 = 1.0./t17;
    g = reshape([t18.*(t15+t16).*-4.0+1.0,t18.*(t8+xi3).*4.0,t18.*(xi2-(xi1.*xi3)./2.0).*-4.0,0.0,t18.*(xi3-(xi1.*xi2)./2.0).*-4.0,t18.*(t14+t16).*-4.0+1.0,t18.*(t11+xi1).*4.0,0.0,t18.*(t9+xi2).*4.0,t18.*(xi1-(xi2.*xi3)./2.0).*-4.0,t18.*(t14+t15).*-4.0+1.0,0.0,t18.*xi4.*(t2./4.0+1.0).*4.0+t18.*xi6.*(t6+t12).*4.0-t18.*xi5.*(t7-t10).*4.0,t18.*xi5.*(t3./4.0+1.0).*4.0+t18.*xi4.*(t7+t10).*4.0-t18.*xi6.*(t5-t13).*4.0,t18.*xi6.*(t4./4.0+1.0).*4.0+t18.*xi5.*(t5+t13).*4.0-t18.*xi4.*(t6-t12).*4.0,1.0],[4,4]);

end

