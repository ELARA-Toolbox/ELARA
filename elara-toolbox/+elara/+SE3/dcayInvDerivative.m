function DdcayInv = dcayInvDerivative(eta, X)
    % Directional derivative of the inverse of the right-trivialized derivative
    % D_eta dcay (X), where this is dcay(eta) differentiated in direction X
    arguments
        eta (6,1)
        X   (6,:)
    end
    % om = eta(1:3);
    % v = eta(4:6);
    % om_hat = skewSO3(om);
    % v_hat = skewSO3(v);
    eta1 = eta(1,:);
    eta2 = eta(2,:);
    eta3 = eta(3,:);
    eta4 = eta(4,:);
    eta5 = eta(5,:);
    eta6 = eta(6,:);

    nDirections = size(X, 2);

    DdcayInv = zeros(6,6,nDirections);
    for i=1:nDirections
        % x = X(1:3, i);
        % y = X(4:6, i);
        % x_hat = skewSO3(x);
        % y_hat = skewSO3(y);
        % 
        % DdcayInv(:,:, i) = 0.5 * [0.5*(om*x.' + x*om.') - x_hat, zeros(3,3);
        %     -(eye(3)-0.5*om_hat)*y_hat + 0.5*x_hat*v_hat, -x_hat];

        % More efficient implementation
        x1 = X(1,i);
        x2 = X(2,i);
        x3 = X(3,i);
        x4 = X(4,i);
        x5 = X(5,i);
        x6 = X(6,i);
        t2 = x1./2.0;
        t3 = x2./2.0;
        t4 = x3./2.0;
        t5 = x4./2.0;
        t6 = x5./2.0;
        t7 = x6./2.0;
        t8 = (eta1.*x2)./4.0;
        t9 = (eta2.*x1)./4.0;
        t10 = (eta1.*x3)./4.0;
        t11 = (eta3.*x1)./4.0;
        t12 = (eta1.*x4)./4.0;
        t13 = (eta2.*x3)./4.0;
        t14 = (eta3.*x2)./4.0;
        t15 = (eta4.*x1)./4.0;
        t16 = (eta2.*x5)./4.0;
        t17 = (eta5.*x2)./4.0;
        t18 = (eta3.*x6)./4.0;
        t19 = (eta6.*x3)./4.0;
        t20 = -t2;
        t21 = -t3;
        t22 = -t4;
        t23 = -t12;
        t24 = -t15;
        t25 = -t16;
        t26 = -t17;
        t27 = -t18;
        t28 = -t19;
        DdcayInv(:,:,i) = reshape([eta1.*t2,t8+t9+t22,t3+t10+t11,t25+t26+t27+t28,-t7+(eta1.*x5)./4.0+(eta5.*x1)./4.0,t6+(eta1.*x6)./4.0+(eta6.*x1)./4.0,t4+t8+t9,eta2.*t3,t13+t14+t20,t7+(eta2.*x4)./4.0+(eta4.*x2)./4.0,t23+t24+t27+t28,-t5+(eta2.*x6)./4.0+(eta6.*x2)./4.0,t10+t11+t21,t2+t13+t14,eta3.*t4,-t6+(eta3.*x4)./4.0+(eta4.*x3)./4.0,t5+(eta3.*x5)./4.0+(eta5.*x3)./4.0,t23+t24+t25+t26,0.0,0.0,0.0,0.0,t22,t3,0.0,0.0,0.0,t4,0.0,t20,0.0,0.0,0.0,t21,t2,0.0],[6,6]);

    end
end