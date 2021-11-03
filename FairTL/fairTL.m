import casadi.*   
addpath('../MiscFunctions');
addpath('../SmoothRobustness/basic');
addpath('DL');
addpath('Fairness');

clc; clear all; close all;
global PLOTIT; PLOTIT = true;

ndrones = 5;
num_iters = 20;

params = generate_problem(ndrones);     % Generate horizons, goals, etc
horizons_min = ones(1, ndrones);

results = cell(1, num_iters);
disp('Starting at');
disp(datetime('now'))
for i=1:num_iters
    disp(i)
    disp(datetime('now'))
    % Loop until we have a feasible solution based on FbL
    while 1
        disp('Attempting to solve standard FbL');
        %% Generate initial positions
        params.p0 = random_p0_generator(params.map, params.obs, params.N_drones);
        % break;
        
        % Save params used for this solution
        results{i}.params = params;

        %% Solve FbL
        % DL = max(horizon)
        params.DL = max(params.H_drones) * ones(size(params.H_drones));

        % Solve problem with params
        results{i}.FbL = solve_fair_trajectory(params);

        % Exit loop if feasible solution found
        if results{i}.FbL.negative_rob < 0
            
            disp('Standard FbL solved');
            break
        end        
    end
    
    %% Solve FairFly1
    disp('Solving FairFly1');
    DL_set = DLSET(horizons_min, params.H_drones, 1);
    results{i}.FairFly1 = solve_fairTL_problem(params, DL_set);
    
    %% Solve FairFly2
    disp('Solving FairFly2');
    DL_set = DLSET(horizons_min, params.H_drones, 2);
    results{i}.FairFly2 = solve_fairTL_problem(params, DL_set);
end

disp('Finished at');
disp(datetime('now'))