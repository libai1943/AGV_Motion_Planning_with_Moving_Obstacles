% ==============================================================================
% Source Codes for "Real-Time Trajectory Planning for AGV in the Presence
% of Moving Obstacles: A First-Search-Then-Optimization Approach".
% ==============================================================================
%
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
% If there are inquiries, please contact libai@zju.edu.cn
%
% ==============================================================================
clear all
close all
clc

%% ====================== Basic setup ======================
global Robs R_ego max_phy max_v min_v
global num_frame time_horizon x_horizon y_horizon
global time_grid_scale x_grid_scale y_grid_scale
global M F G parent
global colorpool

R_ego = 2;   % Specify the radius of the circular ego-AGV
max_phy = 0.35; % Max steering angle of ego-AGV
max_v = 1; % Max velocity of ego-AGV
min_v = 0; % Min velocity of ego-AGV

time_horizon = 40.0; % The time ranges from 0 sec to 40.0 sec
x_horizon = 40.0;    % The x axis ranges from 0 m to 40.0 m
y_horizon = 10.0;    % The y axis ranges from 0 m to 10.0 m

time_grid_scale = 41; % 41 abstracted grids are used to represent the time_horizon
x_grid_scale = 41;    % 41 abstracted grids are used to represent the x_horizon
y_grid_scale = 11;    % 11 abstracted grids are used to represent the y_horizon

num_frame = round(8 * time_horizon); % Video illustration precision
M = zeros(x_grid_scale, y_grid_scale, time_grid_scale); % M will mark the 3D nodes that are occupied by moving obstacles

A = [1, (y_grid_scale + 1) / 2, 1]; % Node A is the starting point
B = [x_grid_scale, (y_grid_scale + 1) / 2, time_grid_scale]; % Node B is the terminal point, i.e. the goal, which the AGV should try to reach, although it may not be able to reach it.

%% ====================== Generation and specification of the moving obstacles ======================
num_obs = 3; % Specify the number of moving obstacles
% Generate as many as num_obs moving obstacles with the precision of num_frame
world_obs_tracklist = obs_gen(num_obs);
M = Mprocessing(M, world_obs_tracklist);

%% ====================== Rough Trajectory Planning Layer using 3D A* Method ======================
F = zeros(size(M));   % F = G + H
G = zeros(size(M));   % G

% x-y-t expansion changes. Note that the reverse driving is igored in this
% work. There is no substantial distinction if more motion directions are
% added. We consider 4 directions just because they are likely to appear
% for the on-road vehicles.
motion = [
    1, 0;
    0, 0;
    1, -1;
    1, 1;];
motion = [motion, ones(4,1)];

openlist = [A];  % Add the starting point A in the openlist OPEN
closelist = [];  % Initialize the closed list CLOSED as empty

parent = cell(size(M));   % Initialize the parent node
inOpen = false(size(M));   % Check if one node is in OPEN
inClose = false(size(M));  % Check if one node is in CLOSED

inOpen(A(1),A(2),A(3)) = true;
G(A(1),A(2),A(3)) = 0;
F(A(1),A(2),A(3)) = hn(A,B);
[index, min_node] = minInOpen(openlist);  % Find the one in OPEN with minimum F value

astar_is_success = 1; % flag that tells if the graph search would be successful
while (min_node(3) ~= B(3)) % Terminal condition we define : if the AGV survives until the last moment, it would be fine, even though it does not arrive at the goal node B
    % Remove the current concerned node from OPEN, and then add it to CLOSED
    openlist(index,:) = [];
    inOpen(min_node(1), min_node(2), min_node(3)) = false;
    closelist = [closelist; min_node];
    inClose(min_node(1), min_node(2), min_node(3)) = true;
    
    for i = 1 : size(motion, 1)
        temp = min_node + motion(i,:);
        if ((temp(1) <= x_grid_scale) && (temp(1) > 0) && (temp(2) <= y_grid_scale) && (temp(2) > 0) && (temp(3) <= time_grid_scale) && (temp(3) > 0))
            if ((M(temp(1),temp(2),temp(3)) ~= 1) && (inClose(temp(1),temp(2),temp(3)) == false))
                if (inOpen(temp(1),temp(2),temp(3)) == 0)
                    parent{temp(1),temp(2),temp(3)} = min_node;
                    openlist = [openlist; temp];
                    G(temp(1),temp(2),temp(3)) = gn(temp);
                    F(temp(1),temp(2),temp(3)) = G(temp(1),temp(2),temp(3)) + hn(temp,B);
                    inOpen(temp(1), temp(2),temp(3)) = true;
                else
                    gnn = norm(min_node - temp) + G(min_node(1), min_node(2), min_node(3)); %
                    if (gnn < G(temp(1),temp(2),temp(3)))
                        parent{temp(1),temp(2),temp(3)} = min_node;
                    end
                end
            end
        end
    end
    if (length(openlist) == 0)
        astar_is_success = 0;
        disp('[RUN_ME] No route could be found -> OPEN is empty.')
        break;
    else
        [index, min_node] = minInOpen(openlist);
    end
end
B = min_node;

% If the 3D A* search process succeeds, begin to prepare for the precise
% optimization stage.
if (astar_is_success)
    route = [B];
    t = parent{B(1),B(2),B(3)};
    while t(1)~=A(1) || t(2)~=A(2) || t(3)~=A(3)
        route = [route; t];
        t = parent{t(1),t(2),t(3)};
    end
    route = [route; A];
    route = flip(route,1);
    % The following 3 variables record the rough trajectory derived by 3D
    % A* method.
    x_rough = route(:,1);
    y_rough = route(:,2);
    z_rough = route(:,3);
end

%% ====================== Precise Trajectory Planning Layer using Interior-Point Method ======================
if (astar_is_success)
    % Set the number of finite elements in the precise optimization stage,
    % and write this in a file (because we do not know API between AMPL and
    % matlab, sorry..)
    NE = 80;
    delete('NE');
    fid = fopen('NE', 'w');
    fprintf(fid,'%g', NE);
    fclose(fid);
    
    % Write num_obs as a file
    delete('NumObs');
    fid = fopen('NumObs', 'w');
    fprintf(fid,'%g', num_obs);
    fclose(fid);
    
    % Write the obstacle radius information in a file, note the last
    % element records the radius of the ego-agv
    delete('Radius_obs');
    fid = fopen('Radius_obs', 'w');
    for ii = 1 : num_obs
        fprintf(fid,'%g %f \r\n', ii, Robs(ii));
    end
    fprintf(fid,'%g %f \r\n', ii+1, R_ego);
    fclose(fid);
    
    % Write the obstacles' movements in a file
    obs2file(world_obs_tracklist, NE);
    % Write the initial guess for NLP solving
    rough_trajectory = route2trajectory(route);
    initial_guess(rough_trajectory, NE);
end

NLP_is_success = 0;
if (astar_is_success)
    !ampl r0.run
    load flag.txt
    NLP_is_success = flag;
end

if ((NLP_is_success)&&(astar_is_success)) % If NLP solving succeeds
    load x.txt
    load y.txt
    load t.txt
    load p.txt
    precise_trajectory = [x', y', t', p'];
    precise_trajectory2 = [precise_resample(precise_trajectory(:,1)', num_frame)', precise_resample(precise_trajectory(:,2)', num_frame)', precise_resample(precise_trajectory(:,3)', num_frame)', precise_resample(precise_trajectory(:,4)', num_frame)'];
end

%% ====================== Static illustration ======================
if ((NLP_is_success)&&(astar_is_success))
    draw_six_figures;
end
