function qc = quick_feas_check(p0,optParams,d)
%p0
%   initial position of drone
% d
%   drone index

H = optParams.DL(d);
% [ goal #; a; b ]  --> eventually_{a,b} (x_d in goal #)
% so ixgoal is the goal #
ixgoal          = optParams.goal_drones(1, d);
noisylicious    = 0*optParams.goal{ixgoal}.ds;

cvx_solver sedumi
cvx_begin quiet

variable ww(H+1,3)
variable v(H,3)

minimise sum(ww(:,1)) %sum(ww(:,1)) makes opt fast, 0 works too

ww(1,:) == p0';
for i = 2:H+1
    ww(i,:) == ww(i-1,:) + v(i-1,:);
    ww(i,:)>=optParams.map.boundary(1:3);
    ww(i,:)<=optParams.map.boundary(4:6);
    v(i-1,:)<=optParams.max_per_axis*ones(3,1)';
    v(i-1,:)>=-optParams.max_per_axis*ones(3,1)';
end


ww(H+1,:)' >= optParams.goal{ixgoal}.lb - noisylicious;
ww(H+1,:)' <= optParams.goal{ixgoal}.ub + noisylicious;
%ww(H+1,:) == optParams.goal{goal_d(1)}.stop' + (0.2*rand(3,1)' - 0.1)*(optParams.N_drones>1); %peturb a bit
cvx_end

if strcmp(cvx_status , 'Infeasible')
    qc = false;
else
    qc = true;
end