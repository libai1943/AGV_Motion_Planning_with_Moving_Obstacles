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
%   This function is used for the generation of moving obstacles in the world space.
%
% ==============================================================================
function obs_tracklist = obs_gen(num_obs)

global colorpool num_frame Robs x_horizon y_horizon
colorpool = rand(num_obs,3); % For illustration
Robs = 0.5 + 0.5 * rand(1, num_obs); % Specify the radius of each moving obstacle, which ranges in [0.5, 1.0]

for ii = 1 : num_obs
    x = rand(1,4) .* x_horizon;
    y = rand(1,4) .* y_horizon;
    val = spcrv([[x(1) x x(end)];[y(1) y y(end)]], 3);
    x = val(1,:);
    y = val(2,:);
    ind = find(x < 0); x(ind) = 0;
    ind = find(x > x_horizon); x(ind) = x_horizon;
    ind = find(y < 0); y(ind) = 0;
    ind = find(y > y_horizon); y(ind) = y_horizon;
    obs_tracklist(ii, :, 1) = precise_resample(x, num_frame);
    obs_tracklist(ii, :, 2) = precise_resample(y, num_frame);
end