function f = getSE3FunctionsNumeric
    %% Get function handles for numeric SE3 Functions
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    eyeF   = @eye;
    zerosF = @zeros;

    f = struct;
    f.eye = eyeF;
    f.zeros = zerosF;

    f.skewSO3      = @skewSO3;
    f.expSO3Screw  = @expSO3Screw;
    f.expSE3Screw  = @expSE3Screw;
    f.caySO3       = @caySO3;
    f.caySE3       = @caySE3;
    f.cayInvSO3    = @cayInvSO3;
    f.cayInvSE3    = @cayInvSE3;
    f.cayRTDSO3    = @cayRTDSO3;
    f.cayRTDInvSO3 = @cayRTDInvSO3;
    f.cayRTDSE3    = @cayRTDSE3;
    f.cayRTDInvSE3 = @cayRTDInvSE3;

    f.lAdSE3       = @lAdSE3;
    f.lAdSE3Inv    = @lAdSE3Inv;
    f.sadSE3       = @sadSE3;
end

%% Numeric Functions
function R = expSO3Screw( om, theta )
    %% Exponential map for SO(3) with constant rotation axis
    arguments
        % Rotation axis 
        om       (3,1)
        % Magnetude / angle
        theta   (1,1)
    end
    omh = skewSO3(om);
    eyeF = getEye(theta);
    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omh*sin(theta) + omh^2*(1-cos(theta));
end
function [R,x] = expSE3Screw( om, v, theta )
    %% Exponential map for SE(3) with constant screw axis
    arguments
        % Screw axis / twist (rotational and translational parts)
        om       (3,1)
        v        (3,1)

        % Magnetude / angle
        theta   (1,1)
    end
    omh = skewSO3(om);

    eyeF = getEye(theta);

    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omh*sin(theta) + omh^2*(1-cos(theta));

    % Formula (2.36), p. 42 in [MLS94]
    x = (eyeF(3) - R)*omh*v + om*om.'*v*theta;
end

function [omega, v] = cayInvSE3( R, x )
    %% Inverse Cayley map for SE(3)
    arguments
        R (3,3)
        x (3,1)
    end
    omegaH = 2 / (1 + trace(R) ) * (R - R.');
    omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
    v  = 2 * ( (R + eye(3)) \ x );
end
function T = cayRTDSE3( om, v )
    %% Right-Trivialized Derivative of the Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end

    omH = skewSO3(om);
    vH  = skewSO3(v);

    T = zeros(6, 6);
    T(1:3, 1:3) = 2 / (4 + om.'*om) * ( 2*eye(3) + omH );
    T(1:3, 4:6) = zeros(3,3);
    T(4:6, 1:3) = 1 / (4 + om.'*om) * vH * ( 2*eye(3) + omH );
    T(4:6, 4:6) = eye(3) + ( 1 / (4 + om.'*om) * ( 2*omH + omH^2 ) );
end

function T = cayRTDInvSE3( om, v )
    %% Inverse Right-Trivialized Derivative of the Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end

    omH = skewSO3( om );
    vH  = skewSO3( v );

    T = zeros(6, 6);
    T(1:3, 1:3) = eye(3) - (1/2 * omH) + (1/4 * (om * om.') );
    T(1:3, 4:6) = zeros(3,3);
    T(4:6, 1:3) = -1/2 * (eye(3) - 1/2 * omH ) * vH;
    T(4:6, 4:6) = eye(3) -1/2 * omH;
end
function T = cayRTDSO3(om)
    %% Right-Trivialized Derivative of the Cayley map for SO(3)
    arguments
        om (3,1)
    end
    omH = skewSO3(om);
    T =  2 / (4 + om.'*om) * ( 2*eye(3) + omH );
end
function T = cayRTDInvSO3(om)
    %% Inverse Right-Trivialized Derivative of the Cayley map for SO(3)
    arguments
        om (3,1)
    end
    omH = skewSO3( om );
    T = eye(3) - (1/2 * omH) + (1/4 * (om * om.') );
end
function R = caySO3(om)
    %% Cayley map for SO(3)
    arguments
        om (3,1)
    end
    omH = skewSO3(om);
    R = eye(3) + 4 / (4 + om.'*om) * (omH + ( omH*omH / 2 ));
end
function [R,x] = caySE3( om, v )
    %% Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end
    omH = skewSO3(om);
    R = caySO3(om);
    x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.') ) * v;
end

function A = lAdSE3(R,x)
    %% Ad operator in matrix form
    A = [
        R,            zeros(3,3);
        skewSO3(x)*R, R;
        ];
end

function A = lAdSE3Inv(R,x)
    %% Inverse Ad operator in matrix form
    A = [
        R.',             zeros(3,3);
        -R.'*skewSO3(x), R.'
        ];
end
function Z = sadSE3(om,v)
    %% small ad representation (6x6 matrix) of an element of se3
    arguments
        om (3,1)
        v  (3,1)
    end
    Z = [
        skewSO3(om), zeros(3,3);
        skewSO3(v),  skewSO3(om);
        ];
end