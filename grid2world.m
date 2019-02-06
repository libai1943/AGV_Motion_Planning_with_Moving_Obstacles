% ==============================================================================
%   Copyright (C) 2019 Bai Li
%   Users are suggested to cite the following article when utilizing the
%   source codes. Bai Li et al., "Real-Time Trajectory Planning for AGV in
%   the Presence of Moving Obstacles: A First-Search-Then-Optimization
%   Approach", 2019 IEEE International Conference on Advanced Robotics and
%   Mechatronics (ICARM), 2019.
%
%   License GNU General Public License v3.0
% ==============================================================================
%
%   This function is used to convert a point in the x-y-t grid state space
%   to the world map. 
%
% ==============================================================================
function [x,y] = grid2world(xg, yg)
global x_horizon y_horizon x_grid_scale y_grid_scale

rate_x = (xg - 1) / (x_grid_scale - 1);
rate_y = (yg - 1) / (y_grid_scale - 1);
x = x_horizon * rate_x;
y = y_horizon * rate_y;