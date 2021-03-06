
% --------------------------------------------------------
% User defined desired path.
% Should be continuous and consist of lines in x-direction
% and curves. Curves should be sufficiently large for
% the robot to traverse them without leaving the path.
% --------------------------------------------------------

global curve section_partition num_sections

syms x

% Example path
curve = {@(x) 0, @(x) sqrt(4 - (x-2.2)^2) - 2, @(x) -sqrt(4 - (x-6.2)^2) - 2, @(x) -4};
% Interconnection of path sections along x
section_partition = [-10, 2.2, 4.2, 6.2];
num_sections = 4;