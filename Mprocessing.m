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
%   This function is used to record the nodes that are occupied by the
%   moving obstacles in M.
%
% ==============================================================================
function M = Mprocessing(M, world_obs_tracklist)

global Robs R_ego num_frame time_grid_scale

num_obs = size(world_obs_tracklist,1);
index = round(linspace(1, num_frame, time_grid_scale));

for ii = 1 : time_grid_scale
    for jj = 1 : num_obs
        R = Robs(1, jj) + R_ego;
        center_x = world_obs_tracklist(jj, index(ii),1);
        center_y = world_obs_tracklist(jj, index(ii),2);
        for deg = 0 : 6 : 360
            for kk = 0 : 50
                angle = deg / 180 * pi;
                r = kk / 50 * R;
                x = center_x + r * cos(angle);
                y = center_y + r * sin(angle);
                [ox, oy] = world2grid(x, y);
                M(ox, oy, ii) = 1;
            end
        end
    end
end