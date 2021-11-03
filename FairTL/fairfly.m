%% start reach avoid example for a multiquads with eth tracker++

% Only need to do the following once
% Setup CVX
% addpath('~/Documents/MATLAB/cvx/')
% cvx_setup
% CASADI setup
% addpath('~/Documents/MATLAB/casadi-osx-matlabR2015a-v3.4.0/')

% Clear console, close plots, and clear all variables
close all;clear;

% addpath('../MiscFunctions');

global PLOTIT; PLOTIT = true;

% Import requirements
import casadi.*   
addpath('../MiscFunctions');
addpath('../SmoothRobustness/basic');
addpath('DL');
addpath('Fairness');


disp('Starting at');
disp(datetime('now'))

%% Setup problem
disp('Initializing...');
tic;
% nb drones
D               = 5;
% Sets up parameters for problem, like N_drones, horizons,
% map/obstacles/goals
params          = generate_problem(D);
% max nb DL vectors to be tried
maxiterations   = 10000;
toc;

%% Set up DL search set
% Get initial DL set
horizons_min    = ones(1, D);
DL_set          = DLSET(horizons_min, params.H_drones, 2);

%% Initialize
params.negative_rob = inf;      % robustness = -inf
iter            = 1;            % track how many solves it takes for timing
feasibleFair    = [];
prevFairest     = [];
%% Solve
while (iter < maxiterations) 
    %% Search for fairness
    % Try next fairest length vector
    % Use first index of the search list (ordered by fairness)
    dlNextFairest           = DL_set.fairest(prevFairest);
    params.DL               = dlNextFairest;
    results = solve_fair_trajectory(params);
    
    % Remove failures
    if results.negative_rob > 0 || isnan(results.negative_rob)
        DL_set      = DL_set.remove_smaller_dls(params.DL);
        prevFairest = dlNextFairest;
    else
        feasibleFair = params.DL;
        break
    end
    iter = iter + 1;
    
    %% Pick a random point for the sake of reducing search space
    % Pick random length tuple
    params.DL                   = DL_set.kinda_random_tuple();
    % Check if feasible
    results   = solve_fair_trajectory(params);
    % if infeasible, remove smaller tuples
    if results.negative_rob > 0 || isnan(results.negative_rob)
        DL_set = DL_set.remove_smaller_dls(params.DL);
    else
        % else, remove all less fair tuples
        feasibleFair    = params.DL;
        fvalue          = fairness2(params.DL, DL_set.min_horizons, DL_set.max_horizons);
        DL_set          = DL_set.remove_less_fair_dls(params.DL, fvalue);
    end        
    iter = iter + 1;
end

%% Save results
if ~isempty(feasibleFair)
    success = true;
else
    success = false;
end

disp('Finished at');
disp(datetime('now'))

save("f2_d" + num2str(params.N_drones) + "_h" + replace(num2str(params.H_drones), "  ", "-") + ".mat",...
     'params', 'results', 'success');

%% Plot results
disp('Plotting...');
if(PLOTIT)
    pause
    pause(1);
    plot_results(params, results);
end
