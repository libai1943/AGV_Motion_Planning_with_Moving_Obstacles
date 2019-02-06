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
%   This function is used to calculate h(s) in A start based method.
%
% ==============================================================================
function v = hn(point,endp)
v = abs(point(2)-endp(2)) + 10 * abs(point(1)-endp(1)) + abs(point(3)-endp(3));