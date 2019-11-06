% User defined specifications

global n l m q0

n = 4; % number of links

m = 1; % link mass

l = 1; % link length

q0    = zeros(n+2,1); % initial joint positions
% q0(1) = pi/6;
% q0(2) = -pi/2;
% q0(3) = pi/3;
% q0(4) = pi/2;

q0(1) = pi/2;
q0(2) = -pi/2;
q0(4) = -pi/4;
q0(6) = -0.8;