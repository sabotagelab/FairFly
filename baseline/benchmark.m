function [num_fail, num_success, run_times, setup_time] = benchmark(N, num_iters)

tic
%clear
obst = [-.1, -.1, -.1; .1, .1, .1];
boundary = [-.5 -.5 -.5; .5 .5 .5];
min_dist = 0.1;                     % Minimum separation

H = 4;                              % Horizon
N_per_T = 10;                       % Time steps per second
%N = 30;                             % N: #drones,
D = 3;                              % D: #dimensions (x,y,etc)

W = sdpvar(N, H+1, D);              % Waypoint position opt variable
V = sdpvar(N, H+1, D);              % Waypoint velocity opt variable
S = sdpvar(N, D);                   % Parameter for starting position
G = sdpvar(N, D);                   % Parameter for goal position

M = 50;                             % Large number for ensuring separation
T = 1;                              % Time between waypoints
M1 = (1/2*T^5)*[90 0 -15*T^2;-90*T 0 15*T^3;30*T^2 0 -3*T^4];
dT = T/N_per_T:T/N_per_T:T;
da = 0;
dv = 0; %start and stop from/at rest

% Constraints
Fsep = [];
Fbounds = [];
Fobst = [];

%% Mutual separation on intermediate splines
for i = 1:N
    for j = i+1:N
        for t = 1:H
            B = binvar(N_per_T,D*2);
            Fsep = [Fsep, sum(B, 2) == ones(N_per_T, 1)];
            for d = 1:D
                dp_di = W(i,t+1,d) - W(i,t,d) - T*V(i,t,d);
                al_di = M1(1,:)*[dp_di;dv;da];
                be_di = M1(2,:)*[dp_di;dv;da];
                gam_di = M1(3,:)*[dp_di;dv;da];
                X_i = al_di/120*dT.^5 + be_di/24*dT.^4 + ...
                      gam_di/6*dT.^3 + W(i,t,d) + dT*V(i,t,d);

                dp_dj = W(j,t+1,d) - W(j,t,d) - T*V(j,t,d);
                al_dj = M1(1,:)*[dp_dj;dv;da];
                be_dj = M1(2,:)*[dp_dj;dv;da];
                gam_dj = M1(3,:)*[dp_dj;dv;da];
                X_j = al_dj/120*dT.^5 + be_dj/24*dT.^4 + ...
                      gam_dj/6*dT.^3 + W(j,t,d) + dT*V(j,t,d);

                for k = 1:N_per_T
                    Fsep = [Fsep,X_i(k)-X_j(k) <= -min_dist+M*(1-B(k,2*d)), ...
                                 X_i(k)-X_j(k) >= min_dist-M*(1-B(k,2*d-1)) ];
                end
            end
        end
    end
end

%% Obstacle avoidance/map boundary constraints on intermediate splines
max_w = boundary(2,:);
for i = 1:N
    for t = 1:H
        B = binvar(N_per_T,D*2);
        Fobst = [Fobst, sum(B, 2) == ones(N_per_T, 1)];
        for d = 1:D
            dp_di = W(i,t+1,d) - W(i,t,d) - T*V(i,t,d);
            al_di = M1(1,:)*[dp_di;dv;da];
            be_di = M1(2,:)*[dp_di;dv;da];
            gam_di = M1(3,:)*[dp_di;dv;da];
            X_i = al_di/120*dT.^5 + be_di/24*dT.^4 + ...
                  gam_di/6*dT.^3 + W(i,t,d) + dT*V(i,t,d);

            for k = 1:N_per_T
                Fobst = [Fobst,X_i(k) <= obst(1,d)+M*(1-B(k,2*d)), ...
                               X_i(k) >= obst(2,d)-M*(1-B(k,2*d-1)) ];
                Fbounds = [Fbounds, boundary(1,d) <= X_i(k) <= boundary(2,d)];
            end
            
        end
    end
end

        

%% Bounds
max_w = boundary(2,:);
max_v = ones(D, 1) * 0.751;
max_a = ones(D, 1) * 1;

K1_T = (90/48)*(1/T) - (90/12)*(1/T) +(30/4)*(1/T);
aa = (90/4)*(1/T^5);
bb = -(90/2)*(1/T^4);
cc = (30/2)*(1/T^3);
tp1 = (-bb+sqrt(bb^2-4*aa*cc))/(2*aa);
tp2 = (-bb-sqrt(bb^2-4*aa*cc))/(2*aa);
t_prime = tp1*(tp1>=0)*(tp1<=T) + tp2*(tp2>=0)*(tp2<=T); %pick the right one
K2_tprime = (90/12)*(t_prime^3)/(T^5) - (90/4)*(t_prime^2)/(T^4) + ...
    (30/2)*(t_prime^1)/(T^3);

max_v = ones(D, 1) * 0.751;
max_a = ones(D, 1) * 1;

for n = 1:N             % iterate over drones
    % Initial position and end position
    Fbounds = [Fbounds, reshape(W(n,  1,:), [1 D]) == S(n,:), ...
                        reshape(W(n,end,:), [1 D]) == G(n,:), ...
                        V(n,  1,:) == zeros(D, 1), ...
                        V(n,end,:) == zeros(D, 1)];
    
    for k = 1:H         % iterate over horizon
        for d = 1:D     % iterate over dimensions
            dp_d = W(n,k+1,d) - W(n,k,d);
            vd_k = V(n,k+1,d); vd_km1 = V(n,k,d);

            % constants for all 3 axes
            al_d = M1(1,:)*[dp_d-T*vd_km1;0;da];
            be_d = M1(2,:)*[dp_d-T*vd_km1;0;da];
            gam_d = M1(3,:)*[dp_d-T*vd_km1;0;da];
            vf_d = (al_d/24)*T^4 + (be_d/6)*T^3 + (gam_d/2)*T^2 + vd_km1;

            v_d = K1_T*dp_d + (1 - T*K1_T)*vd_km1;
            a_d = K2_tprime*dp_d - T*K2_tprime*vd_km1;
            

            Fbounds = [Fbounds, boundary(1,d) <= W(n, k, d) <= boundary(2,d), ...
                                -max_v(d) <= v_d <= max_v(d), ...
                                -max_a(d) <= a_d <= max_a(d), ...
                                vd_k == vf_d ];
        end
    end
end

%% Solver setup
solver_options = sdpsettings('verbose', 1, ...
                             'solver', 'gurobi', ...
                             'gurobi.TimeLimit', 2000, ...
                             'gurobi.MIPGap', 0.1, ...
                             'gurobi.MIPGapAbs', 0.01, ...
                             'gurobi.NumericFocus', 3, ...
                             'gurobi.SolutionLimit', Inf,...
                             'cachesolvers',1);

%objective = norm(sum(abs(V), [1,3]), 1) + norm(sum(abs(W), [1, 3]), 1); % Minimize U, V
objective = []; % Just want feasible solution

P = optimizer([Fbounds, Fsep, Fobst], objective, solver_options, {S, G}, {W, V});
setup_time = toc;
%%
num_fail = 0;
num_success = 0;
run_times = zeros(num_iters, 1);

for i = 1:num_iters
    if mod(i, 10) == 0
        fprintf("iteration %d\n", i);
    end
    
    [start, goal] = generate_config(N, min_dist);
    tic
    [res, ~, ~, ~, ~, diag] = P(start, goal);
    run_times(i) = toc;
    
    if diag.problem ~= 0
        num_fail = num_fail + 1;
        fprintf('Solution was not feasible!\n')
    else
        num_success = num_success + 1;
    end
end
%%
%x = spline_w(res{1}, res{2}, T, M1, dv, da);
%plot_drones(x, res{1}, obst, boundary);

%disp("done")
end
