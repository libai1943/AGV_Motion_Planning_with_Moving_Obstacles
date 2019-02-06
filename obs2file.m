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
%   This function is used for the generation of a file that records the
%   trajectories of the moving obstacles for NLP solving.
%
% ==============================================================================
function obs2file(obs, NE)

num_obs = size(obs, 1);
len = size(obs, 2);
main_index = round(linspace(1, len, NE));
delete('libai_x');
delete('libai_y');
fid_x = fopen('libai_x', 'w');
fid_y = fopen('libai_y', 'w');
for ii = 1 : num_obs
    for jj = 1 : NE
        fprintf(fid_x,'%g  %g  %f \r\n', ii, jj, obs(ii, main_index(jj), 1));
        fprintf(fid_y,'%g  %g  %f \r\n', ii, jj, obs(ii, main_index(jj), 2));
    end
end
fclose(fid_x);
fclose(fid_y);