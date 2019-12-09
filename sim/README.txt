
Simulator for a simple 2D snake robot interacting with point-shaped
obstacles on a flat surface.

Author: Atussa Koushan, NTNU
Last changed: 13.11.2019
------------------------------------------------------------------------

To change the specifications for the robot, go to the init_snake.m file.
To change the specifications for the obstacles, go to the init_obstacles
file.


Running the simulator:

1. Run initialization.m
2. Run main.m

- If the obstacle specifications are changed, it is sufficient to re-run
  init_obstacles.m.
- If the snake specifications are changed, initialization.m has to be 
  re-run.


Model assumptions:

- The model assumes that the robot can be in contact with maximum one
  obstacle per link.
- The model assumes that the length and mass of every link is the same.