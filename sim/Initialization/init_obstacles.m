
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles obstacle_radius


num_obstacles = 0;
obstacle_radius = 0.1;
obstacle_coords = zeros(num_obstacles, 2);

obstacle_coords = [2.7 -0.2;
                   1.8 0.2];

% % case 2.3 jaa med h= 0.005
% obstacle_coords = [-0.1 -0.1 %men juks
%                    0.6 -0.1;
%                    1.6 0.1;
%                    3.2 -0.33];

% % Gets stuck with h=0.005
% obstacle_coords = [
%                    0.6 -0.1;
%                    1.6 0.1;
%                    3.2 -0.33];

% % case 2.1
% obstacle_coords = [
%                    0.8 -0.08;
%                    1.6 0.08;
%                    3.3 -0.3];

% % case 2.2
% obstacle_coords = [
%                    1.5 0.3;
%                    2.5 -0.5;
%                    3.6 -0.8];

