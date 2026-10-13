function RTDInv_dt = dcayInvDerivative(eta,X)
    %DCAYINVDERIVATIVE Directional derivative of the inverse Cayley tangent map.
    %   RTDINV_DT = elara.SE3.dcayInvDerivative(ETA,X) evaluates
    %   D_ETA dcayInv(X(:,i)) for every direction X(:,i).  With N directions,
    %   RTDINV_DT has size 6-by-6-by-N.  A single direction retains the legacy
    %   6-by-6 result.
    %
    %   This is the multi-direction form previously provided by DcayRTDInv.

    arguments
        eta (6,1)
        X   (6,:)
    end

    nDirections = size(X,2);
    RTDInv_dt = zeros(6,6,nDirections,'like',eta);
    for iDirection = 1:nDirections
        RTDInv_dt(:,:,iDirection) = dcayInvDerivativeSingle( ...
            eta,X(:,iDirection));
    end
end

function RTDInv_dt = dcayInvDerivativeSingle(in1,in2)
    % Generated single-direction expression retained for the hot path.

    xi1 = in1(1,:);
    xi2 = in1(2,:);
    xi3 = in1(3,:);
    xi4 = in1(4,:);
    xi5 = in1(5,:);
    xi6 = in1(6,:);
    xi_dot1 = in2(1,:);
    xi_dot2 = in2(2,:);
    xi_dot3 = in2(3,:);
    xi_dot4 = in2(4,:);
    xi_dot5 = in2(5,:);
    xi_dot6 = in2(6,:);
    t2 = xi_dot1./2.0;
    t3 = xi_dot2./2.0;
    t4 = xi_dot3./2.0;
    t5 = xi_dot4./2.0;
    t6 = xi_dot5./2.0;
    t7 = xi_dot6./2.0;
    t8 = (xi1.*xi_dot2)./4.0;
    t9 = (xi2.*xi_dot1)./4.0;
    t10 = (xi1.*xi_dot3)./4.0;
    t11 = (xi3.*xi_dot1)./4.0;
    t12 = (xi1.*xi_dot4)./4.0;
    t13 = (xi2.*xi_dot3)./4.0;
    t14 = (xi3.*xi_dot2)./4.0;
    t15 = (xi4.*xi_dot1)./4.0;
    t16 = (xi2.*xi_dot5)./4.0;
    t17 = (xi5.*xi_dot2)./4.0;
    t18 = (xi3.*xi_dot6)./4.0;
    t19 = (xi6.*xi_dot3)./4.0;
    t20 = -t2;
    t21 = -t3;
    t22 = -t4;
    t23 = -t12;
    t24 = -t15;
    t25 = -t16;
    t26 = -t17;
    t27 = -t18;
    t28 = -t19;
    RTDInv_dt = reshape([t2.*xi1,t8+t9+t22,t3+t10+t11,t25+t26+t27+t28,-t7+(xi1.*xi_dot5)./4.0+(xi5.*xi_dot1)./4.0,t6+(xi1.*xi_dot6)./4.0+(xi6.*xi_dot1)./4.0,t4+t8+t9,t3.*xi2,t13+t14+t20,t7+(xi2.*xi_dot4)./4.0+(xi4.*xi_dot2)./4.0,t23+t24+t27+t28,-t5+(xi2.*xi_dot6)./4.0+(xi6.*xi_dot2)./4.0,t10+t11+t21,t2+t13+t14,t4.*xi3,-t6+(xi3.*xi_dot4)./4.0+(xi4.*xi_dot3)./4.0,t5+(xi3.*xi_dot5)./4.0+(xi5.*xi_dot3)./4.0,t23+t24+t25+t26,0.0,0.0,0.0,0.0,t22,t3,0.0,0.0,0.0,t4,0.0,t20,0.0,0.0,0.0,t21,t2,0.0],[6,6]);
end
