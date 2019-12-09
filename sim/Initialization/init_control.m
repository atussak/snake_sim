
% --------------------------------------------------------
% User defined control parameters.
% (For computed torque control)
% --------------------------------------------------------

global kp kd max_tau N m

% Controller gains
kp = eye(N)*0.8;
kd = eye(N)*1.8;

%zeta = 1; %critically damped
zeta = 2; %overdamped
%zeta = 0.3; %underdamped

M_0 = eye(N)*m;
kp = eye(N)*0.4;
kd = 2*zeta*sqrtm(M_0*kp);

% Torque saturation value
max_tau = 1;