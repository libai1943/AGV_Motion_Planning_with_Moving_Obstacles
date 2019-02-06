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
%   This function is used to extract the minimum-f val element from OPEN.
%
% ==============================================================================
function [index,mc] = minInOpen(open)
global F
mv =  inf;
for ii = 1 : size(open,1)
    v = F(open(ii,1),open(ii,2),open(ii,3));
    if (v < mv)
        mv = v;
        mc = open(ii,:);
        index = ii;
    end
end