
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles


num_obstacles = 6;
obstacle_coords = zeros(num_obstacles, 2);

obstacle_coords = [
                   0.8 -0.15;
                   1.6 0.15;
                   1.8 -0.15;
                   2.3 0.15;
                   3.3 -0.2;
                   3.5 -0.6];
               
               