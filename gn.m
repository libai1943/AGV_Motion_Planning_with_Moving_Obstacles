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
%   This function is used to calculate g(s) in A start based method.
%
% ==============================================================================
function v = gn(point)

global G parent
pr = parent{point(1),point(2),point(3)};
ed = norm(pr-point);
v = G(pr(1),pr(2),pr(3)) + ed;