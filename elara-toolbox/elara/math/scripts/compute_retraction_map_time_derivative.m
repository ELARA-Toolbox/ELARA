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
RTD = cayRTDSE3(xi_t)
RTD_dt = diff(RTD, t)
RTD_dt = subs(RTD_dt, diff(xi_t,t), xi_dot);
RTD_dt = subs(RTD_dt, xi_t, xi)
%RTD_dt = simplify(RTD_dt)

% Save as file
matlabFunction( ...
    RTD_dt, ...
    'File', '../cayRTDSE3dt', ...
    'Vars', {xi, xi_dot} ...
    );

cayRTDSE3dt(rand(6,1), rand(6,1))
timeit(@() cayRTDSE3dt(rand(6,1), rand(6,1)))


%% Inverse

RTDInv = cayRTDInvSE3(xi_t)
RTDInv_dt = diff(RTDInv, t)
RTDInv_dt = subs(RTDInv_dt, diff(xi_t,t), xi_dot);
RTDInv_dt = subs(RTDInv_dt, xi_t, xi)
%RTDInv_dt = simplify(RTDInv_dt)


% Save as file
matlabFunction( ...
    RTDInv_dt, ...
    'File', '../cayRTDInvSE3dt', ...
    'Vars', {xi, xi_dot} ...
    );
cayRTDInvSE3dt(rand(6,1), rand(6,1))
timeit(@() cayRTDInvSE3dt(rand(6,1), rand(6,1)))



