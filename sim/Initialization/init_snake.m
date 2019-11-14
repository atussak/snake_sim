
% --------------------------------------------------------
% User defined specifications for the 2D snake robot
% --------------------------------------------------------


global n N l m q0


% Number of links
n = 4;

% Link mass
m = 1;

% Link length
l = 1;

% Initial joint positions and virtual variable values
q0    = zeros(N,1);


% --------------------------------------------------------
% Control
% --------------------------------------------------------

global max_tau

% For torque control
max_tau = 1;