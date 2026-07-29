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
res = elara.SO3.cayInv( elara.SO3.cay( omega ) ) - omega;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map SO3 / Test 2
R = eul2rotm( rand (1,3) );
res = elara.SO3.cay( elara.SO3.cayInv( R ) ) - R;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map SE3 / Test 1
xi = rand(6,1);
res = elara.SE3.cayInv( elara.SE3.cay(xi) ) - xi;
%disp(res)
assert( max(abs(res(:))) <= tol );

%% Cayley map SE3 / Test 2
R = eul2rotm( rand (1,3) );
g = elara.SE3.matrix( R, rand(3,1) );
res = elara.SE3.cay ( elara.SE3.cayInv( g ) ) - g;
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map right-triv. derivative SE3 / Test 1
xi = rand(6,1);
res = elara.SE3.dcay(xi) * elara.SE3.dcayInv(xi) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );


%% Cayley map right-triv. derivative Test SE3 / Test 2
xi = rand(6,1);
res = elara.SE3.dcayInv(xi) * elara.SE3.dcay(xi) - eye(6);
%disp(res)
assert( max(abs(res(:))) <= tol );
