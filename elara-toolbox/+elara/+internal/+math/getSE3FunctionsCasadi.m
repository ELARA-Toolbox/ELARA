function f = getSE3FunctionsCasadi
    %% Get CasADi functions for the SE(3) functions
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    eyeF   = @casadi.SX.eye;
    zerosF = @casadi.SX.zeros;

    f = struct;
    f.eye = eyeF;
    f.zeros = zerosF;

    funOpts = struct();
    funOpts.cse = true; % Common subexpression elimination

    %% Symbolic variables
    % Note: The duplicate variables with -2 are used in definitions that
    % Include previously defined CasADi functions

    omS     = casadi.SX.sym('om', 3, 1);
    omS2    = casadi.SX.sym('om2', 3, 1);
    vS      = casadi.SX.sym('v', 3, 1);
    vS2     = casadi.SX.sym('v2', 3, 1);
    thetaS  = casadi.SX.sym('theta', 1, 1);
    thetaS2 = casadi.SX.sym('theta2', 1, 1);

    RS      = casadi.SX.sym('R', 3, 3);
    RS2     = casadi.SX.sym('R2', 3, 3);
    xS      = casadi.SX.sym('x', 3, 1);
    xS2     = casadi.SX.sym('x2', 3, 1);
 
    c3S     = casadi.SX.sym('c', 3, 1); % Only used for SO(3) skew function

    %% SO(3) skew
    f.SO3.skew = casadi.Function('SO3_skew', ...
        {c3S}, {[ 0, -c3S(3), c3S(2); c3S(3), 0, -c3S(1); -c3S(2), c3S(1), 0]}, ...
        {'omega'}, {'omegaHat'}, funOpts);

    omHS  = f.SO3.skew(omS);
    omHS2 = f.SO3.skew(omS2);
    vHS   = f.SO3.skew(vS);
    xHS   = f.SO3.skew(xS);


    %% expSO3Screw
    % Exponential map for SO(3) with constant rotation axis

    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omHS*sin(thetaS) + omHS^2*(1-cos(thetaS));

    f.expSO3Screw = casadi.Function('expSO3Screw', ...
        {omS, thetaS}, {R}, ...
        {'omega', 'theta'}, {'R'}, funOpts);

    clear R


    %% SE(3) screw exponential
    % Exponential map for SE(3) with constant screw axis

    % Formula (2.14), p. 28 in [MLS94]
    R = f.expSO3Screw(omS2, thetaS2);

    % Formula (2.36), p. 42 in [MLS94]
    x = (eyeF(3) - R)*omHS2*vS2 + omS2*omS2.'*vS2*thetaS2;

    f.SE3.expScrew = casadi.Function('SE3_expScrew', ...
        {omS2, vS2, thetaS2}, {R, x}, ...
        {'omega', 'v', 'theta'}, {'R', 'x'}, funOpts);

    clear R x


    %% SO(3) Cayley map
    % Cayley map for SO(3)
    R = eyeF(3) + 4 / (4 + omS.'*omS) * (omHS + ( omHS*omHS / 2 ));
    f.SO3.cay = casadi.Function('SO3_cay', ...
        {omS}, {R}, ...
        {'omega'}, {'R'}, funOpts);

    clear R

    %% SO(3) inverse Cayley map
    % Inverse cayley map for SO(3)
    omegaH = 2 / (1 + trace(RS) ) * (RS - RS.');
    omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
    f.SO3.cayInv = casadi.Function('SO3_cayInv', ...
        {RS}, {omega}, ...
        {'R'}, {'omega'}, funOpts);

    clear omegaH omega


    %% SE(3) Cayley map
    % Cayley map for SE(3): cay : se(3) -> SE(3)
    % Source: [Dem+14, p.10], eq. 19

    R = f.SO3.cay(omS2);
    x = ( 4 / (4 + omS2.'*omS2) ) * ( eyeF(3) + (1/2) * omHS2 + 1/4 * (omS2*omS2.') ) * vS2;

    f.SE3.cay = casadi.Function('SE3_cay', ...
        {omS2, vS2}, {R, x}, ...
        {'omega', 'v'}, {'R', 'x'}, funOpts);

    clear R x

    %% SE(3) inverse Cayley map
    % Inverse cayley map for SE(3)
    omega = f.SO3.cayInv(RS2);
    v  = 2 * ( (RS2 + eyeF(3)) \ xS2 );

    f.SE3.cayInv = casadi.Function('SE3_cayInv', ...
        {RS2, xS2}, {omega, v}, ...
        {'R', 'x'}, {'omega', 'v'}, funOpts);

    clear omega v

    %% SO(3) right-trivialized Cayley derivative
    % Right-Trivialized Derivative of the Cayley map for SO(3) in Matrix
    % form
    % Source: [KM11], eq. 31, [Dem+14] eq. 17
    T = 2 / (4 + omS.'*omS) * ( 2*eyeF(3) + omHS );

    f.SO3.dcay = casadi.Function('SO3_dcay', ...
        {omS}, {T}, ...
        {'omega'}, {'T'}, funOpts);

    clear T

    %% SO(3) inverse right-trivialized Cayley derivative
    % Inverse Right-Trivialized Derivative of the Cayley map for SO(3) in
    % matrix form

    TInv = eyeF(3) - ((1/2) * omHS) + ((1/4) * (omS * omS.') );

    f.SO3.dcayInv = casadi.Function('SO3_dcayInv', ...
        {omS}, {TInv}, ...
        {'omega'}, {'TInv'}, funOpts);
    
    clear TInv


    %% SE(3) right-trivialized Cayley derivative
    % Right-Trivialized Derivative of the Cayley map for SE(3) in Matrix
    % form

    T = [
        2 / (4 + omS.'*omS) * ( 2*eyeF(3) + omHS ), ...
        zerosF(3,3); ...
        1 / (4 + omS.'*omS) * vHS * ( 2*eyeF(3) + omHS ), ...
        eyeF(3) + ( 1 / (4 + omS.'*omS) * ( 2*omHS + omHS^2 ) ) ...
        ];

    f.SE3.dcay = casadi.Function('SE3_dcay', ...
        {omS, vS}, {T}, ...
        {'omega', 'v'}, {'T'}, funOpts);

    clear T


    %% SE(3) inverse right-trivialized Cayley derivative
    % Inverse Right-Trivialized Derivative of the Cayley map for SE(3) in
    % matrix form

    TInv = [
        eyeF(3) - ((1/2) * omHS) + ((1/4) * (omS * omS.') ), ...
        zerosF(3,3); ...
        -(1/2) * (eyeF(3) - (1/2) * omHS ) * vHS, ...
        eyeF(3) - (1/2) * omHS ...
        ];

    f.SE3.dcayInv = casadi.Function('SE3_dcayInv', ...
        {omS, vS}, {TInv}, ...
        {'omega', 'v'}, {'TInv'}, funOpts);
    
    clear TInv


    %% SE(3) Ad
    % Ad operator in matrix form
    Ad = [
        RS,            zerosF(3,3);
        xHS*RS, RS;
        ];
    f.SE3.Ad = casadi.Function('SE3_Ad', ...
        {RS, xS}, {Ad}, ...
        {'R', 'x'}, {'Ad'}, funOpts);

    clear Ad


    %% SE(3) inverse Ad
    % Inverse Ad operator in matrix form
    AdInv = [
        RS.',             zerosF(3,3);
        -RS.'*xHS, RS.'
        ];
    f.SE3.AdInv = casadi.Function('SE3_AdInv', ...
        {RS, xS}, {AdInv}, ...
        {'R', 'x'}, {'AdInv'}, funOpts);

    clear AdInv

    %% SE(3) small ad
    % (small) ad operator in matrix form
    sad = [
        omHS, zerosF(3,3);
        vHS,  omHS;
        ];
   f.SE3.smallAd = casadi.Function('SE3_smallAd', ...
        {omS, vS}, {sad}, ...
        {'omega', 'v'}, {'ad'}, funOpts);

    clear sad
end
