
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles


num_obstacles = 6;
obstacle_coords = zeros(num_obstacles, 2);

obstacle_coords = [2.8 0.1;
                   1.8 0.1;
                   3.4 -0.4;
                   3.9 -1.5;
                   3.5 -2.4;
                   3.7 -3.2];
               
               