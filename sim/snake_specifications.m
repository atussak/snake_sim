% User defined specifications

global n l m q0

n = 3; % number of links

m = 1; % link mass

l = 1; % link length

q0    = zeros(n+2,1); % initial joint positions
q0(1) = pi/6;
q0(2) = -pi/2;
q0(3) = pi/3;