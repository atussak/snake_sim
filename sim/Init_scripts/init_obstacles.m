
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles obstacle_radius


num_obstacles = 3;
obstacle_radius = 0.1;
obstacle_coords = zeros(num_obstacles, 2);


% Example
obstacle_coords = [1.5 0.3;
                   2.5 -0.5;
                   3.6 -0.8];

