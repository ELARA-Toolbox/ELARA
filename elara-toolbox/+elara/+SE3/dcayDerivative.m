function Ddcay = dcayDerivative(eta, X)
    % Directional derivative of the right-trivialized derivative
    % D_eta dcay (X), where this is dcay(eta) differentiated in direction X
    arguments
        eta (6,1)
        X   (6,:)
    end
    om = eta(1:3);
    % v = eta(4:6);
    % om_hat = skewSO3(om);
    % v_hat = skewSO3(v);

    nDirections = size(X, 2);

    dcay = elara.SE3.dcay(eta);
    factor = 1/(4 + om.'*om);

    dcay1_1 = dcay(1);
    dcay1_2 = dcay(7);
    dcay1_3 = dcay(13);
    dcay2_1 = dcay(2);
    dcay2_2 = dcay(8);
    dcay2_3 = dcay(14);
    dcay3_1 = dcay(3);
    dcay3_2 = dcay(9);
    dcay3_3 = dcay(15);
    dcay4_1 = dcay(4);
    dcay4_2 = dcay(10);
    dcay4_3 = dcay(16);
    dcay4_4 = dcay(22);
    dcay4_5 = dcay(28);
    dcay4_6 = dcay(34);
    dcay5_1 = dcay(5);
    dcay5_2 = dcay(11);
    dcay5_3 = dcay(17);
    dcay5_4 = dcay(23);
    dcay5_5 = dcay(29);
    dcay5_6 = dcay(35);
    dcay6_1 = dcay(6);
    dcay6_2 = dcay(12);
    dcay6_3 = dcay(18);
    dcay6_4 = dcay(24);
    dcay6_5 = dcay(30);
    dcay6_6 = dcay(36);
    eta1 = eta(1,:);
    eta2 = eta(2,:);
    eta3 = eta(3,:);
    eta4 = eta(4,:);
    eta5 = eta(5,:);
    eta6 = eta(6,:);


    Ddcay = zeros(6,6,nDirections);
    for i=1:nDirections
        % x = X(1:3, i);
        % y = X(4:6, i);
        % x_hat = skewSO3(x);
        % y_hat = skewSO3(y);
        %
        % Ddcay(:,:, i) = factor* ([2*x_hat, zeros(3,3); ...
        %     y_hat*(2*eye(3)+om_hat) + v_hat*x_hat, 2*x_hat + om*x.' + x*om.'] ...
        %     - 2*om.'*x*dcay);

        % More efficient implementation

        x1 = X(1,i);
        x2 = X(2,i);
        x3 = X(3,i);
        x4 = X(4,i);
        x5 = X(5,i);
        x6 = X(6,i);
        t2 = eta1.*x2;
        t3 = eta2.*x1;
        t4 = eta1.*x3;
        t5 = eta3.*x1;
        t6 = eta1.*x4;
        t7 = eta2.*x3;
        t8 = eta3.*x2;
        t9 = eta4.*x1;
        t10 = eta2.*x5;
        t11 = eta5.*x2;
        t12 = eta3.*x6;
        t13 = eta6.*x3;
        t14 = x1.*2.0;
        t15 = x2.*2.0;
        t16 = x3.*2.0;
        t17 = x4.*2.0;
        t18 = x5.*2.0;
        t19 = x6.*2.0;
        t20 = eta1.*t14;
        t21 = eta2.*t15;
        t22 = eta3.*t16;
        t23 = t20+t21+t22;
        mt1 = [-dcay1_1.*factor.*t23,factor.*(t16-dcay2_1.*t23),-factor.*(t15+dcay3_1.*t23),-factor.*(t10+t11+t12+t13+dcay4_1.*t23),factor.*(t19-dcay5_1.*t23+eta2.*x4+eta4.*x2),-factor.*(t18+dcay6_1.*t23-eta3.*x4-eta4.*x3),-factor.*(t16+dcay1_2.*t23),-dcay2_2.*factor.*t23,factor.*(t14-dcay3_2.*t23),-factor.*(t19+dcay4_2.*t23-eta1.*x5-eta5.*x1),-factor.*(t6+t9+t12+t13+dcay5_2.*t23),factor.*(t17-dcay6_2.*t23+eta3.*x5+eta5.*x3),factor.*(t15-dcay1_3.*t23),-factor.*(t14+dcay2_3.*t23),-dcay3_3.*factor.*t23,factor.*(t18-dcay4_3.*t23+eta1.*x6+eta6.*x1),-factor.*(t17+dcay5_3.*t23-eta2.*x6-eta6.*x2)];
        mt2 = [-factor.*(t6+t9+t10+t11+dcay6_3.*t23),0.0,0.0,0.0,factor.*(t20-dcay4_4.*t23),factor.*(t2+t3+t16-dcay5_4.*t23),factor.*(t4+t5-t15-dcay6_4.*t23),0.0,0.0,0.0,factor.*(t2+t3-t16-dcay4_5.*t23),factor.*(t21-dcay5_5.*t23),factor.*(t7+t8+t14-dcay6_5.*t23),0.0,0.0,0.0,factor.*(t4+t5+t15-dcay4_6.*t23),factor.*(t7+t8-t14-dcay5_6.*t23),factor.*(t22-dcay6_6.*t23)];
        Ddcay(:,:,i) = reshape([mt1,mt2],6,6);

    end
end
