function params = generate_problem(ndrones)
global PLOTIT;

if nargin<1
    ndrones = 5;
end
% #drones per goal. Every this many drones get one goal.
dpg = 5;

%% Load Map and plot goal/obstacle
map_name = 'map0.txt';

%% Setup problem parameters
H_range             = [5, 10];                      % Horizon range
params.N_drones     = ndrones;                      % Number of drones
params.num_goals    = ceil(params.N_drones / dpg);  % Number of goals
% Horizon of each drone
params.H_drones     = randi(H_range, 1, params.N_drones);
% H seconds %works with 5 for d=2, but use small C and recompute. 6 is ok
params.h            = 1/20;                         %dt

% Generate random goals in map
[map, obs] = plot_env({{}}, map_name);
close all;
% Minimum 3 units between each goal
goal_locs = random_p0_generator(map, obs, params.num_goals, 3); 
% Generate num_goals distinct random colors, that are not white or red
params.drone_colors = distinguishable_colors(params.num_goals, ...
                                            {'w', 'r'}, ...
                                            @(x) colorspace('RGB->Lab',x));

goal = cell(params.num_goals, 1);
% Setup the goal zone
for i=1:params.num_goals
    %problem.goal{i}.stop = [1.75;1.75;0.75]; %map end point
    goal{i}.stop = goal_locs(:, i); %map end point
    goal{i}.ds = .25; %thickeness
    goal{i}.lb = goal{i}.stop-goal{i}.ds;
    goal{i}.ub = goal{i}.stop+goal{i}.ds;
    goal{i}.poly = Polyhedron('lb',goal{i}.lb,'ub',goal{i}.ub);
    goal{i}.col = params.drone_colors(i, :); %'green';
end
params.goal = goal;

% Which goal each drone is aiming for
params.goal_drones = ceil((1:params.N_drones)/dpg);

% Plot the map with obstacles, goals, boundaries
[params.map, params.obs] = plot_env(params.goal, map_name);
if ~PLOTIT
    close all;
end

% Generate random initial positions
params.p0 = random_p0_generator(params.map, params.obs, params.N_drones);

% Plot the initial positions of drones
if PLOTIT
    for i=1:params.N_drones
        plot3(params.p0(1, i), params.p0(2, i), params.p0(3, i), 'o', 'Color', params.goal{ceil(i/dpg)}.col);
    end
end

% Setup Casadi parameters

T = 1;                          %1s duration of motion
params.T = T;
params.M1 = (1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;
    60*T^2 -24*T^3 3*T^4];
params.da = 0;
params.dv = 0;                  %start and stop from/at rest
params.max_per_axis = 1;
params.N_per_T = T/params.h;    % Number of discrete steps per time step
params.d_min = 0.1;             % min mutual separation

pause(0.1);
end

