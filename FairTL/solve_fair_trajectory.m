function results = solve_fair_trajectory(params)
% Determine whether the inner robustness maximization is solvable with the
% given dl vector
% qcPassed
%   true if quick_feas_check passed, false otherwise

qcPassed = true;

%% Quick feasibility check
for d=1:params.N_drones
    if ~quick_feas_check(params.p0(:,d),params,d)
        qcPassed = false;
        w_opt = nan;      
        results.negative_rob = inf;
        return;
    end
end

%% traj gen constraints in casadi
% Recalculate some parameters with new DL
params.Nsteps_drones = params.DL*params.N_per_T;
% Clen: number of X,Y,Z for each time step in (horizon + 1)
params.Clen = 3*(params.DL+1);
results.DL = params.DL;

%disp('Formulating in Casadi...');
tic;
[solver, lbw, ubw, lbg, ubg] = setup_casadi_problem(params);
results.times.casadi = toc;

%% Get init guess via lp
%disp('Getting init solution...');
tic;
w0 = [];
for i = 1:params.N_drones
    % Get init waypoints guess for each drone, only up to its horizon
    temp = get_init_waypoints_variable_trajectory(params.p0(:,i),params, i);
    w0 = [w0;temp];
end
results.times.init_sol = toc;

%% Solve the NLP
%disp('Solving...');
tic;
sol = solver('x0',w0,'lbx', lbw, 'ubx', ubw,...
    'lbg', lbg, 'ubg', ubg);
results.w_opt = full(sol.x);
[results.negative_rob,~,~,~] = cost_fair_trajectory_Ndrones(results.w_opt,params);
results.times.solve = toc;
end
