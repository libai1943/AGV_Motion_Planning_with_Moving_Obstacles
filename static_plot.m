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

function static_plot(trajectory, route)
plot(route(:,1)', route(:,2)', 'b', 'LineWidth', 2);
hold on
plot(trajectory(:,1)', trajectory(:,2)', 'y','LineWidth',2);
legend('Rough trajectory derived by 3D A* algorithm','Precise trajectory optimized by IPM');
axis equal
xlabel('x axis / m');
ylabel('y axis / m');