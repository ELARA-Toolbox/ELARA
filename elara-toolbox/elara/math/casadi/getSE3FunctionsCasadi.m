function f = getSE3FunctionsCasadi
    %% Get Casadi functions for the SE3 functions
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
    % include previously defined casadi functions

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
 
    c3S     = casadi.SX.sym('c', 3, 1); % Only used for skewSO3 function

    %% skewSO3
    f.skewSO3 = casadi.Function('skewSO3', ...
        {c3S}, {[ 0, -c3S(3), c3S(2); c3S(3), 0, -c3S(1); -c3S(2), c3S(1), 0]}, ...
        {'omega'}, {'omegaHat'}, funOpts);

    omHS  = f.skewSO3(omS);
    omHS2 = f.skewSO3(omS2);
    vHS   = f.skewSO3(vS);
    xHS   = f.skewSO3(xS);


    %% expSO3Screw
    % Exponential map for SO(3) with constant rotation axis

    % Formula (2.14), p. 28 in [MLS94]
    R = eyeF(3) + omHS*sin(thetaS) + omHS^2*(1-cos(thetaS));

    f.expSO3Screw = casadi.Function('expSO3Screw', ...
        {omS, thetaS}, {R}, ...
        {'omega', 'theta'}, {'R'}, funOpts);

    clear R


    %% expSE3Screw
    % Exponential map for SE(3) with constant screw axis

    % Formula (2.14), p. 28 in [MLS94]
    R = f.expSO3Screw(omS2, thetaS2);

    % Formula (2.36), p. 42 in [MLS94]
    x = (eyeF(3) - R)*omHS2*vS2 + omS2*omS2.'*vS2*thetaS2;

    f.expSE3Screw = casadi.Function('expSE3Screw', ...
        {omS2, vS2, thetaS2}, {R, x}, ...
        {'omega', 'v', 'theta'}, {'R', 'x'}, funOpts);

    clear R x


    %% caySO3
    % Cayley map for SO(3)
    R = eyeF(3) + 4 / (4 + omS.'*omS) * (omHS + ( omHS*omHS / 2 ));
    f.caySO3 = casadi.Function('caySO3', ...
        {omS}, {R}, ...
        {'omega'}, {'R'}, funOpts);

    clear R

    %% cayInvSO3
    % Inverse cayley map for SO(3)
    omegaH = 2 / (1 + trace(RS) ) * (RS - RS.');
    omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
    f.cayInvSO3 = casadi.Function('cayInvSO3', ...
        {RS}, {omega}, ...
        {'R'}, {'omega'}, funOpts);

    clear omegaH omega


    %% caySE3
    % Cayley map for SE(3): cay : se(3) -> SE(3)
    % Source: [Dem+14, p.10], eq. 19

    R = f.caySO3(omS2);
    x = ( 4 / (4 + omS2.'*omS2) ) * ( eyeF(3) + (1/2) * omHS2 + 1/4 * (omS2*omS2.') ) * vS2;

    f.caySE3 = casadi.Function('caySE3', ...
        {omS2, vS2}, {R, x}, ...
        {'omega', 'v'}, {'R', 'x'}, funOpts);

    clear R x

    %% cayInvSE3
    % Inverse cayley map for SE(3)
    omega = f.cayInvSO3(RS2);
    v  = 2 * ( (RS2 + eyeF(3)) \ xS2 );

    f.cayInvSE3 = casadi.Function('cayInvSE3', ...
        {RS2, xS2}, {omega, v}, ...
        {'R', 'x'}, {'omega', 'v'}, funOpts);

    clear omega v

    %% cayRTDSO3
    % Right-Trivialized Derivative of the Cayley map for SO(3) in Matrix
    % form
    % Source: [KM11], eq. 31, [Dem+14] eq. 17
    T = 2 / (4 + omS.'*omS) * ( 2*eyeF(3) + omHS );

    f.cayRTDSO3 = casadi.Function('cayRTDSO3', ...
        {omS}, {T}, ...
        {'omega'}, {'T'}, funOpts);

    clear T

    %% cayRTDInvSO3
    % Inverse Right-Trivialized Derivative of the Cayley map for SO(3) in
    % matrix form

    TInv = eyeF(3) - ((1/2) * omHS) + ((1/4) * (omS * omS.') );

    f.cayRTDInvSO3 = casadi.Function('cayRTDInvSO3', ...
        {omS}, {TInv}, ...
        {'omega'}, {'TInv'}, funOpts);
    
    clear TInv


    %% cayRTDSE3
    % Right-Trivialized Derivative of the Cayley map for SE(3) in Matrix
    % form

    T = [
        2 / (4 + omS.'*omS) * ( 2*eyeF(3) + omHS ), ...
        zerosF(3,3); ...
        1 / (4 + omS.'*omS) * vHS * ( 2*eyeF(3) + omHS ), ...
        eyeF(3) + ( 1 / (4 + omS.'*omS) * ( 2*omHS + omHS^2 ) ) ...
        ];

    f.cayRTDSE3 = casadi.Function('cayRTDSE3', ...
        {omS, vS}, {T}, ...
        {'omega', 'v'}, {'T'}, funOpts);

    clear T


    %% cayRTDInvSE3
    % Inverse Right-Trivialized Derivative of the Cayley map for SE(3) in
    % matrix form

    TInv = [
        eyeF(3) - ((1/2) * omHS) + ((1/4) * (omS * omS.') ), ...
        zerosF(3,3); ...
        -(1/2) * (eyeF(3) - (1/2) * omHS ) * vHS, ...
        eyeF(3) - (1/2) * omHS ...
        ];

    f.cayRTDInvSE3 = casadi.Function('cayRTDInvSE3', ...
        {omS, vS}, {TInv}, ...
        {'omega', 'v'}, {'TInv'}, funOpts);
    
    clear TInv


    %% lAdSE3
    % Ad operator in matrix form
    Ad = [
        RS,            zerosF(3,3);
        xHS*RS, RS;
        ];
    f.lAdSE3 = casadi.Function('lAdSE3', ...
        {RS, xS}, {Ad}, ...
        {'R', 'x'}, {'Ad'}, funOpts);

    clear Ad


    %% lAdSE3Inv
    % Inverse Ad operator in matrix form
    AdInv = [
        RS.',             zerosF(3,3);
        -RS.'*xHS, RS.'
        ];
    f.lAdSE3Inv = casadi.Function('lAdSE3Inv', ...
        {RS, xS}, {AdInv}, ...
        {'R', 'x'}, {'AdInv'}, funOpts);

    clear AdInv

    %% sadSE3
    % (small) ad operator in matrix form
    sad = [
        omHS, zerosF(3,3);
        vHS,  omHS;
        ];
   f.sadSE3 = casadi.Function('sadSE3', ...
        {omS, vS}, {sad}, ...
        {'omega', 'v'}, {'ad'}, funOpts);

    clear sad
end
