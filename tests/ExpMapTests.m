% Unit tests for the Exponential map functions
%
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

% Test tolerance
tol = 1e-12;

% Make randomized test inputs reproducible
rng default;


%% Exponential map SO3 / Test 1
omega = rand(3,1);
res = expInvSO3( expSO3( omega ) ) - omega;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / Test 2
R = eul2rotm( rand (1,3) );
res = expSO3( expInvSO3( R ) ) - R;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / zero input
R = expSO3(zeros(3,1));
res = R - eye(3);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / zero input inverse
omega = expInvSO3(eye(3));
assert( max(abs(omega(:))) <= tol );


%% Exponential map SE3 / Test 1
xi = rand(6,1);
res = expInvSE3( expSE3(xi) ) - xi;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / Test 2
R = eul2rotm( rand (1,3) );
g = SE3Matrix( R, rand(3,1) );
res = expSE3( expInvSE3( g ) ) - g;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / zero input
g = expSE3(zeros(6,1));
res = g - eye(4);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / zero input inverse
xi = expInvSE3(eye(4));
assert( max(abs(xi(:))) <= tol );


%% Exponential map right-triv. derivative SE3 / Test 1
xi = rand(6,1);
res = expRTDSE3(xi) * expRTDInvSE3(xi) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative Test SE3 / Test 2
xi = rand(6,1);
res = expRTDInvSE3(xi) *expRTDSE3(xi) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative SE3 / Zero Input
res = expRTDSE3(zeros(6,1)) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative Test SE3 / Zero Input Inverse
res = expRTDInvSE3(zeros(6,1)) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );
