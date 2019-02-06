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
%   This function is used for the generation of precise sample points.
%
% ==============================================================================
function result = precise_resample(x, num_frame)

NE = length(x);
no_sample_time = round(num_frame / NE) * 3 + 10;
temp_vec = [];
for ii = 2 : NE
    pre_x = x(ii - 1);
    d_x = x(ii) - x(ii - 1);
    for kk = 1 : no_sample_time
        temp_vec = [temp_vec, pre_x + d_x / no_sample_time * (kk - 1)];
    end
end
temp_vec = [temp_vec, x(end)];
main_index = round(linspace(1, length(temp_vec), num_frame));
result = temp_vec(main_index);