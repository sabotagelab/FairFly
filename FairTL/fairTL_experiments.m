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


%% Setup problem
disp('Initializing...');
tic;
% Sets up parameters for problem, like N_drones, horizons,
% map/obstacles/goals
D       = 2;
params  = generate_problem(D);
toc;

%% Set up DL search set
% Get initial DL set
horizons_min    = ones(1,D);
DL_set = DL_reach_avoid(params.H_drones);
sizeDL = size(DL_set,1);

% Sort DL set by fairness
fairness        = fairness2(DL_set, horizons_min, params.H_drones);
[allsorted, U]  = sortrows([fairness DL_set], 'descend');
fairness        = allsorted(:,1);
DL_set          = allsorted(:, 2:end);    
clear allsorted;
% search  will shrink as we discard infeasible lengths
search = 1:sizeDL;     % indexes over fairness 

%% Initialize
params.negative_rob = inf;      % robustness = -inf
iter = 1;                       % track how many solves it takes for timing

%% Solve
while (~isempty(search)) % Loop until explicit break, or search is empty  
    %% Search for fairness
    % Try next fairest length vector
    % Use first index of the search list (ordered by fairness)
    params.DL = DL_set(search(1), :);
    params.DL_attempt{iter} = params.DL;
    results = solve_fair_trajectory(params);
    
    % Remove failures
    if results.negative_rob > 0 || isnan(results.negative_rob)
        search = remove_smaller_dls(DL_set, params.DL, search);       
    else
        success = true;
        break
    end
    iter = iter + 1;
    
    if isempty(search)
        success = false;
        break
    end
    
    %% Search for elimination
    % Take the middle element of remainder ~lexicographical~ fair search
    % Then ~map that to the ordered search space and~ use that DL
    ixhalfway = ceil(length(search)/2);
    params.DL = DL_set(search(ixhalfway), :);
    params.DL_attempt{iter} = params.DL;
    results = solve_fair_trajectory(params);
    
    % Remove failures
    if results.negative_rob > 0 || isnan(results.negative_rob)
        search = remove_smaller_dls(DL_set, params.DL, search);
    % Or remove stuff with less fairness
    else
        % Remove everything after current because they are all less fair
        search = search(1:ixhalfway);
    end
    
   	iter = iter + 1;
end

if(success)
%% Save results
    save("f2_d" + num2str(params.N_drones) + "_h" + replace(num2str(params.H_drones), "  ", "-") + ".mat",...
         'params');

%% Plot results
    disp('Plotting...');
    pause
    plot_results(params, results);
end
