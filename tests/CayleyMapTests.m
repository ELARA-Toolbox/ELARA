% Unit tests for the Cayley-map functions
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

% Test tolerance
tol = 1e-12;

% Make randomized test inputs reproducible
rng default;


%% Cayley map SO3 / Test 1
omega = rand(3,1);
res = cayInvSO3( caySO3( omega ) ) - omega;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map SO3 / Test 2
R = eul2rotm( rand (1,3) );
res = caySO3( cayInvSO3( R ) ) - R;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map SE3 / Test 1
xi = rand(6,1);
res = cayInvSE3( caySE3(xi) ) - xi;
disp(res)
assert( max(abs(res(:))) <= tol );

%% Cayley map SE3 / Test 2
R = eul2rotm( rand (1,3) );
g = SE3Matrix( R, rand(3,1) );
res = caySE3 ( cayInvSE3( g ) ) - g;
disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map right-triv. derivative SE3 / Test 1
xi = rand(6,1);
res = cayRTDSE3(xi) * cayRTDInvSE3(xi) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map right-triv. derivative Test SE3 / Test 2
xi = rand(6,1);
res = cayRTDInvSE3(xi) * cayRTDSE3(xi) - eye(6);
disp(res)
assert( max(abs(res(:))) <= tol );
