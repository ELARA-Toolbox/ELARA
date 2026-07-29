%% Compute the time derivative of the right-trivialized derivaive of the retraction map
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

addpath("../");

syms t
syms xi_t(t) xi xi_dot [6,1]


%% Derivative
RTD = elara.SE3.dcay(xi_t)
RTD_dt = diff(RTD, t)
RTD_dt = subs(RTD_dt, diff(xi_t,t), xi_dot);
RTD_dt = subs(RTD_dt, xi_t, xi)
%RTD_dt = simplify(RTD_dt)

% Save as file
matlabFunction( ...
    RTD_dt, ...
    'File', '../../../+elara/+SE3/dcayDerivative', ...
    'Vars', {xi, xi_dot} ...
    );

elara.SE3.dcayDerivative(rand(6,1), rand(6,1))
timeit(@() elara.SE3.dcayDerivative(rand(6,1), rand(6,1)))


%% Inverse

RTDInv = elara.SE3.dcayInv(xi_t)
RTDInv_dt = diff(RTDInv, t)
RTDInv_dt = subs(RTDInv_dt, diff(xi_t,t), xi_dot);
RTDInv_dt = subs(RTDInv_dt, xi_t, xi)
%RTDInv_dt = simplify(RTDInv_dt)


% Save as file
matlabFunction( ...
    RTDInv_dt, ...
    'File', '../../../+elara/+SE3/dcayInvDerivative', ...
    'Vars', {xi, xi_dot} ...
    );
elara.SE3.dcayInvDerivative(rand(6,1), rand(6,1))
timeit(@() elara.SE3.dcayInvDerivative(rand(6,1), rand(6,1)))



