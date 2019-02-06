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
%   This function is used for the preparation of initial guess: a
%   preliminary step that convert the nodes to their locations in the world
%   map.
%
% ==============================================================================
function trajectory = route2trajectory(route)
global num_frame time_grid_scale

Path = zeros(time_grid_scale, 2);
for ii = 1 : time_grid_scale
    [Path(ii, 1), Path(ii, 2)] = grid2world(route(ii,1), route(ii,2));
end
precise_path_x = precise_resample(Path(:,1)', num_frame)';
precise_path_y = precise_resample(Path(:,2)', num_frame)';
trajectory = [precise_path_x, precise_path_y];