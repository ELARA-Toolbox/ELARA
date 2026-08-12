function f = getSE3FunctionsNumeric
    %% Get function handles for numeric SE(3) functions
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

    f.SO3.skew     = @elara.SO3.skew;
    f.expSO3Screw  = @expSO3Screw;
    f.SE3.expScrew = @expScrewSE3Parts;
    f.SO3.cay      = @elara.SO3.cay;
    f.SO3.cayInv   = @elara.SO3.cayInv;
    f.SO3.dcay     = @elara.SO3.dcay;
    f.SO3.dcayInv  = @elara.SO3.dcayInv;
    f.SE3.cay      = @caySE3Parts;
    f.SE3.cayInv   = @cayInvSE3Parts;
    f.SE3.dcay     = @dcaySE3Parts;
    f.SE3.dcayInv  = @dcayInvSE3Parts;
    f.SE3.Ad       = @AdSE3Parts;
    f.SE3.AdInv    = @AdInvSE3Parts;
    f.SE3.smallAd  = @smallAdSE3Parts;
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
    omh = elara.SO3.skew(om);
    eyeF = elara.internal.math.getEye(theta);
    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omh*sin(theta) + omh^2*(1-cos(theta));
end
function [R,x] = expScrewSE3Parts( om, v, theta )
    %% Exponential map for SE(3) with constant screw axis
    arguments
        % Screw axis / twist (rotational and translational parts)
        om       (3,1)
        v        (3,1)

        % Magnetude / angle
        theta   (1,1)
    end
    omh = elara.SO3.skew(om);

    eyeF = elara.internal.math.getEye(theta);

    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omh*sin(theta) + omh^2*(1-cos(theta));

    % Formula (2.36), p. 42 in [MLS94]
    x = (eyeF(3) - R)*omh*v + om*om.'*v*theta;
end

function [omega, v] = cayInvSE3Parts(R, x)
    %% Inverse Cayley map for SE(3)
    arguments
        R (3,3)
        x (3,1)
    end
    omegaH = 2 / (1 + trace(R) ) * (R - R.');
    omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
    v  = 2 * ( (R + eye(3)) \ x );
end
function T = dcaySE3Parts(om, v)
    %% Right-Trivialized Derivative of the Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end

    omH = elara.SO3.skew(om);
    vH  = elara.SO3.skew(v);

    T = zeros(6, 6);
    T(1:3, 1:3) = 2 / (4 + om.'*om) * ( 2*eye(3) + omH );
    T(1:3, 4:6) = zeros(3,3);
    T(4:6, 1:3) = 1 / (4 + om.'*om) * vH * ( 2*eye(3) + omH );
    T(4:6, 4:6) = eye(3) + ( 1 / (4 + om.'*om) * ( 2*omH + omH^2 ) );
end

function T = dcayInvSE3Parts(om, v)
    %% Inverse Right-Trivialized Derivative of the Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end

    omH = elara.SO3.skew( om );
    vH  = elara.SO3.skew( v );

    T = zeros(6, 6);
    T(1:3, 1:3) = eye(3) - (1/2 * omH) + (1/4 * (om * om.') );
    T(1:3, 4:6) = zeros(3,3);
    T(4:6, 1:3) = -1/2 * (eye(3) - 1/2 * omH ) * vH;
    T(4:6, 4:6) = eye(3) -1/2 * omH;
end
function [R,x] = caySE3Parts(om, v)
    %% Cayley map for SE(3)
    arguments
        om (3,1)
        v  (3,1)
    end
    omH = elara.SO3.skew(om);
    R = elara.SO3.cay(om);
    x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.') ) * v;
end

function A = AdSE3Parts(R, x)
    %% Ad operator in matrix form
    A = [
        R,            zeros(3,3);
        elara.SO3.skew(x)*R, R;
        ];
end

function A = AdInvSE3Parts(R, x)
    %% Inverse Ad operator in matrix form
    A = [
        R.',             zeros(3,3);
        -R.'*elara.SO3.skew(x), R.'
        ];
end
function Z = smallAdSE3Parts(om, v)
    %% Small adjoint representation of an element of se(3)
    arguments
        om (3,1)
        v  (3,1)
    end
    Z = [
        elara.SO3.skew(om), zeros(3,3);
        elara.SO3.skew(v),  elara.SO3.skew(om);
        ];
end
