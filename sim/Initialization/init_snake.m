
% --------------------------------------------------------
% User defined specifications for the 2D snake robot
% --------------------------------------------------------


global n N l m q0


% Number of links
n = 3;

% All virtual and real joints
N = n + 2 + n; % n   : one joint per link
               % + 2 : for the virtual x- and y-coordinates
               % + n : length to every possible obstacle from last joint
               %       (maximum one obstacle per link)

% Link mass
m = 1;

% Link length
l = 1;

% Initial joint positions and virtual variable values
q0    = zeros(N,1);