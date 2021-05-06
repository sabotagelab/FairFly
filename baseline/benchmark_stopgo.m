%function result=benchmark(start, goal, bounds)
%% Args
% start: NxD
% goal:  NxD
% bounds: ... %bounds are the max/min xyz
%%
clear
start = [1 2 3; 4 2 1; 9 2 4];
goal = [5 3 1; 2 1 3; 6 7 5];
H = 6;                  % Horizon
N_per_T = 10;           % Time steps per second
T = H*N_per_T + 1;    % Number of discrete steps
[N, D] = size(start);   % N: num drones, D: num dimensions (x, y, z, etc)

min_dist = 1;           % Minimum separation
M = 20;              % Large number for ensuring separation

W = sdpvar(N, H+1, D);              % Waypoint position opt variable
%V = sdpvar(N, H+1, D);              % Waypoint velocity opt variable
B = binvar(nchoosek(N, 2), T, D*2); % Binary choice opt variable
X = sdpvar(N, T, D);                % Interpolated spline positions

% Constraints
Fspline = [];
Fbin = [];  % Constraints for binary numbers 
Fsep = [];
Fbounds = [];

%% Intermediate Splines
dt = 1; % Time between waypoints
M1 = (1/(dt^5))*[720 -360*dt 60*dt^2;-360*dt 168*dt^2 -24*dt^3;...
    60*dt^2 -24*dt^3 3*dt^4];
dT = dt/N_per_T:dt/N_per_T:dt;
max_per_axis = 1;
da = 0;
dv = 0; %start and stop from/at rest

for n = 1:N
    for t = 1:H
        for d = 1:D
            dp_d = W(n,t+1,d) - W(n,t,d);% - dt*V(n,t,d);
            al_d = M1(1,:)*[dp_d;dv;da];
            be_d = M1(2,:)*[dp_d;dv;da];
            gam_d = M1(3,:)*[dp_d;dv;da];
            
            %X(n, 1+(t-1)*N_per_T:t*N_per_T, d) = (al_d/120)*dT.^5 + ...
            %                                     (be_d/24)*dT.^4 + ...
            %                                     (gam_d/6)*dT.^3 + ...
            %                                     W(n,t,d) + dT*V(n,t,d);
            Fspline = [Fspline, X(n, 2+(t-1)*N_per_T:t*N_per_T+1, d) == ...
                                            ((al_d/120)*dT.^5 + ... 
                                             (be_d/24)*dT.^4 + ...
                                             (gam_d/6)*dT.^3 + ...
                                             W(n,t,d))];% + dT*V(n,t,d))];
        end
    end
end

%% Mutual separation constraints
b = 1;      % Track which number of mutual drone pairs we are on
for i = 1:N
    for j = i+1:N
        for t = 1:T
        Fbin = [Fbin, sum(B(b, t, :)) == 1];    % Only care about 1 dim/dir
            Fsep = [Fsep, (X(i,t,1) - X(j,t,1) >= min_dist - M*(1-B(b,t,1))), ...
                          (X(i,t,1) - X(j,t,1) <= -(min_dist - M*(1-B(b,t,2)))), ...
                          (X(i,t,2) - X(j,t,2) >= min_dist - M*(1-B(b,t,3))), ...
                          (X(i,t,2) - X(j,t,2) <= -(min_dist - M*(1-B(b,t,4)))), ...
                          (X(i,t,3) - X(j,t,3) >= min_dist - M*(1-B(b,t,5))), ...
                          (X(i,t,3) - X(j,t,3) <= -(min_dist - M*(1-B(b,t,6)))) ];
            %for d = 1:D
            %    Fsep = [Fsep ... 
            %       -min_dist + -M*B(b,t,(d*2)) <= X(i,t,d) - X(j,t,d) <=...
            %        min_dist +  M*B(b,t,(d*2)-1) ...
            %           ];
            %end
            %Fsep = [Fsep,(X(i,t,1) - X(j,t,1) >= min_dist) | ...
            %             (X(i,t,2) - X(j,t,2) >= min_dist) | ...
            %             (X(i,t,3) - X(j,t,3) >= min_dist) | ...
            %             (X(i,t,1) - X(j,t,1) <= -min_dist) | ...
            %             (X(i,t,2) - X(j,t,2) <= -min_dist) | ...
            %             (X(i,t,3) - X(j,t,3) <= -min_dist) ];
        end

        b = b + 1;
    end
end

%% Bounds
max_w = ones(D, 1) * 10;
%max_v = ones(D, 1) * 0.751;
%max_a = ones(D, 1) * 1;

%K1_T = (90/48)*(1/dt) - (90/12)*(1/dt) +(30/4)*(1/dt);
%aa = (90/4)*(1/dt^5);
%bb = -(90/2)*(1/dt^4);
%cc = (30/2)*(1/dt^3);
%tp1 = (-bb+sqrt(bb^2-4*aa*cc))/(2*aa);
%tp2 = (-bb-sqrt(bb^2-4*aa*cc))/(2*aa);
%t_prime = tp1*(tp1>=0)*(tp1<=T) + tp2*(tp2>=0)*(tp2<=T); %pick the right one
%K2_tprime = (90/12)*(t_prime^3)/(T^5) - (90/4)*(t_prime^2)/(T^4) + ...
%    (30/2)*(t_prime^1)/(T^3);

for n = 1:N
    % Initial position and end position
    Fbounds = [Fbounds, W(n,1,:) == start(n,:)];%, V(n,1,:) == zeros(D, 1)];
    Fbounds = [Fbounds, W(n,end,:) == goal(n,:)];%, V(n,end,:) == zeros(D, 1)];
    Fbounds = [Fbounds, X(n,1,:) == start(n,:)];%, V(n,1,:) == zeros(D, 1)];
    %Fbounds = [Fbounds, X(n,end,:) == goal(n,:)];%, V(n,end,:) == zeros(D, 1)];
    for k = 1:H
        for d = 1:D
            dp_d = W(n,k+1,d) - W(n,k,d);
            %vd_k = V(n,k+1,d); vd_km1 = V(n,k,d);

            % constants for all 3 axes
            %al_d = M1(1,:)*[dp_d-T*vd_km1;0;da];
            %be_d = M1(2,:)*[dp_d-T*vd_km1;0;da];
            %gam_d = M1(3,:)*[dp_d-T*vd_km1;0;da];
            %vf_d = (al_d/24)*T^4 + (be_d/6)*T^3 + (gam_d/2)*T^2 + vd_km1;

            %v_d = K1_T*dp_d + (1 - T*K1_T)*vd_km1;
            %a_d = K2_tprime*dp_d - T*K2_tprime*vd_km1;

            Fbounds = [Fbounds, -max_w(d) <= W(n, k, d) <= max_w(d), ...
                                -max_per_axis <= dp_d <= max_per_axis ];
                                %-max_v(d) <= v_d <= max_v(d), ...
                                %-max_a(d) <= a_d <= max_a(d), ...
                                %0 <= vd_k-vf_d <= 0 ];
        end
    end
    for k = 1:T
        for d = 1:D
            Fbounds = [Fbounds, -max_w(d) <= X(n,k,d) <= max_w(d)];
        end
    end
end

%% Solver setup
solver_options = sdpsettings('verbose', 1, ...
                             'solver', 'gurobi', ...
                             'gurobi.TimeLimit', 2000, ...
                             'gurobi.MIPGap', 0.1, ...
                             'gurobi.MIPGapAbs', 0.01, ...
                             'gurobi.SolutionLimit', Inf,...
                             'gurobi.NumericFocus', 3, ...
                             'cachesolvers',1);

objective = norm(sum(abs(W), [1,3]), 1);% + norm(sum(abs(V), [1, 3]), 1); % Minimize U, V

optimize([Fbounds, Fspline, Fsep], objective, solver_options);

x = value(X);
w = value(W);
plot_drones(x, w);
disp("done")

%end