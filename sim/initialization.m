
% --------------------------------------------------------
% Run this to set up the workspace for the simulator.
% --------------------------------------------------------

addpath('Init_scripts')
addpath('Control_functions')
addpath('Modeling_functions')
addpath('Visualization_functions')
addpath('Path-following_functions')

clear

init_obstacles
init_snake
init_path
init_control
init_dynamics
init_kinematics
init_contact_jacobians