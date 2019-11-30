
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles obstacle_radius


num_obstacles = 6;
obstacle_radius = 0.1;
obstacle_coords = zeros(num_obstacles, 2);

obstacle_coords = [
                   0.8 -0.12;
                   1.6 0.12;
                   1.8 -8.75;
                   2.3 8.35;
                   3.2 -0.3;
                   3.9 -1.1];
               
               