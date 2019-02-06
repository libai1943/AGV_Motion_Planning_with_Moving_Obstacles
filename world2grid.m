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
%   This function is used to convert a point in the world map to a node in
%   the x-y-t state space.
%
% ==============================================================================

function [xg, yg] = world2grid(x, y)
global x_horizon y_horizon x_grid_scale y_grid_scale

if (x < 0)
    x = 0;
elseif (x > x_horizon)
    x = x_horizon;
end

if (y < 0)
    y = 0;
elseif (y > y_horizon)
    y = y_horizon;
end

rate_x = x / x_horizon;
xg = 1 + round((x_grid_scale - 1) * rate_x);
rate_y = y / y_horizon;
yg = 1 + round((y_grid_scale - 1) * rate_y);