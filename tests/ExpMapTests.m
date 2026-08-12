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
res = elara.SO3.expInv( elara.SO3.exp( omega ) ) - omega;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / Test 2
R = eul2rotm( rand (1,3) );
res = elara.SO3.exp( elara.SO3.expInv( R ) ) - R;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / zero input
R = elara.SO3.exp(zeros(3,1));
res = R - eye(3);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SO3 / zero input inverse
omega = elara.SO3.expInv(eye(3));
assert( max(abs(omega(:))) <= tol );


%% Exponential map SE3 / Test 1
xi = rand(6,1);
res = elara.SE3.expInv( elara.SE3.exp(xi) ) - xi;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / Test 2
R = eul2rotm( rand (1,3) );
g = elara.SE3.matrix( R, rand(3,1) );
res = elara.SE3.exp( elara.SE3.expInv( g ) ) - g;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / zero input
g = elara.SE3.exp(zeros(6,1));
res = g - eye(4);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map SE3 / zero input inverse
xi = elara.SE3.expInv(eye(4));
assert( max(abs(xi(:))) <= tol );


%% Exponential map right-triv. derivative SE3 / Test 1
xi = rand(6,1);
res = elara.SE3.dexp(xi) * elara.SE3.dexpInv(xi) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative Test SE3 / Test 2
xi = rand(6,1);
res = elara.SE3.dexpInv(xi) *elara.SE3.dexp(xi) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative SE3 / Zero Input
res = elara.SE3.dexp(zeros(6,1)) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Exponential map right-triv. derivative Test SE3 / Zero Input Inverse
res = elara.SE3.dexpInv(zeros(6,1)) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );
