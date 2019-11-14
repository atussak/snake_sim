
% --------------------------------------------------------
% User defined specifications.
% Initialize position of obstacles.
% These positions will remain constant through the simulation.
% --------------------------------------------------------


global obstacle_coords num_obstacles


num_obstacles = 4;
obstacle_coords = zeros(num_obstacles, 2);

obstacle_coords(1,1) = 2.8;
obstacle_coords(1,2) = 0.1;

obstacle_coords(2,1) = 1.8;
obstacle_coords(2,2) = 0.1;

obstacle_coords(3,1) = 4;
obstacle_coords(3,2) = -0.9;

obstacle_coords(4,1) = 3.4;
obstacle_coords(4,2) = -0.6;