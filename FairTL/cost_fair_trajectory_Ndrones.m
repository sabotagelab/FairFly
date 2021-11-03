function [negative_rob,xx,yy,zz] = cost_fair_trajectory_Ndrones(w,optParams)
import casadi.*
type_of = isfloat(w); %0 for casadi

C = 30; %const for smooth min/max operation %for 2 drones, use 10 with a period of 5s, casadi is unstable numerically
% C1 = 30; %const for smooth max
dT          = 1/optParams.N_per_T:1/optParams.N_per_T:optParams.T;
dv          = optParams.dv;
da          = optParams.da;
M1          = optParams.M1;
Clen        = optParams.Clen;
N_drones    = optParams.N_drones;
H_formula   = max(optParams.DL) + 1;
H_drones    = optParams.H_drones;
DL          = optParams.DL;
N_per_T     = optParams.N_per_T;
d_min       = optParams.d_min;

if(type_of) %if double input
    temp_x = zeros(N_drones, numel(dT));
    temp_y = zeros(N_drones, numel(dT));
    temp_z = zeros(N_drones, numel(dT));
    xx = zeros(numel(dT)*H_formula+1,N_drones);
    yy = zeros(numel(dT)*H_formula+1,N_drones);
    zz = zeros(numel(dT)*H_formula+1,N_drones);
    rho_unsafe = zeros(N_drones,1);
    rho_goal = zeros(N_drones,1);
    mutual_distances = zeros(numel(dT)*H_formula+1,1);
    
    if(N_drones>1)
    dists = zeros(nchoosek(N_drones,2),1);
    end
else
    %temp_x = MX.sym('temp_x',N_drones, numel(dT));
    temp_x = MX.zeros(N_drones, numel(dT));
    
    %temp_y = MX.sym('temp_y',N_drones, numel(dT));
    temp_y = MX.zeros(N_drones, numel(dT));
    
    %temp_z = MX.sym('temp_z',N_drones, numel(dT));
    temp_z = MX.zeros(N_drones, numel(dT));
    
    % xx = MX.sym('xx',numel(dT)*H_formula+1,N_drones);
    xx = MX.zeros(numel(dT)*H_formula+1,N_drones);    
    % yy = MX.sym('yy',numel(dT)*H_formula+1,N_drones);
    yy = MX.zeros(numel(dT)*H_formula+1,N_drones);    
    % zz = MX.sym('zz',numel(dT)*H_formula+1,N_drones);
    zz = MX.zeros(numel(dT)*H_formula+1,N_drones);

    %rho_unsafe = MX.sym('r_u',N_drones,1);
    rho_unsafe = MX.zeros(N_drones,1);
    %rho_goal = MX.sym('r_u',N_drones,1);
    rho_goal = MX.zeros(N_drones,1);
    %mutual_distances = MX.sym('msep',numel(dT)*H_formula+1,1);
    mutual_distances = MX.zeros(numel(dT)*H_formula+1,1);
    if(N_drones>1)
    % dists = MX.sym('mdist',nchoosek(N_drones,2),1);
    dists = MX.zeros(nchoosek(N_drones,2),1);
    end
end

for d = 1:N_drones
    %init positions
    xx(1,d) = w(1+sum(Clen(1:d-1)));
    yy(1,d) = w(2+sum(Clen(1:d-1)));
    zz(1,d) = w(3+sum(Clen(1:d-1)));
    
    % for k = 1:H_formula
    for k = 1:DL(d)
        
        % dp for all axes
        dp_x = w(k*3+1+sum(Clen(1:d-1))) - w((k-1)*3+1+sum(Clen(1:d-1)));
        dp_y = w(k*3+2+sum(Clen(1:d-1))) - w((k-1)*3+2+sum(Clen(1:d-1)));
        dp_z = w(k*3+3+sum(Clen(1:d-1))) - w((k-1)*3+3+sum(Clen(1:d-1)));
        
        % constants for all 3 axes
        al_x = M1(1,:)*[dp_x;dv;da];
        be_x = M1(2,:)*[dp_x;dv;da];
        gam_x = M1(3,:)*[dp_x;dv;da];
        al_y = M1(1,:)*[dp_y;dv;da];
        be_y = M1(2,:)*[dp_y;dv;da];
        gam_y = M1(3,:)*[dp_y;dv;da];
        al_z = M1(1,:)*[dp_z;dv;da];
        be_z = M1(2,:)*[dp_z;dv;da];
        gam_z = M1(3,:)*[dp_z;dv;da];
        
        temp_x(d,:) = (al_x/120)*dT.^5 + (be_x/24)*dT.^4 + ...
            (gam_x/6)*dT.^3   + w((k-1)*3+1+sum(Clen(1:d-1))); %fix w points
        xx(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_x(d,:)';
        temp_y(d,:) = (al_y/120)*dT.^5 + (be_y/24)*dT.^4 + ...
            (gam_y/6)*dT.^3   + w((k-1)*3+2+sum(Clen(1:d-1))); % fix w points
        yy(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_y(d,:)';
        temp_z(d,:) = (al_z/120)*dT.^5 + (be_z/24)*dT.^4 + ...
            (gam_z/6)*dT.^3   + w((k-1)*3+3+sum(Clen(1:d-1))); %fix w points
        zz(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_z(d,:)'; 
        
    end
    
    states = [xx(:,d), yy(:,d), zz(:,d)];
    % always_{0, H_drone} not unsafe in x y z
    rho_unsafe(d) = alwaysNot(1, states, optParams.obs{1}, 0:DL(d)*numel(dT), C);
    
    % eventually_{0, H_drone} in goal_drone x y z
    goal_d = optParams.goal_drones(d);
    goal_t = min(H_drones(d), DL(d));
    rho_goal(d) = eventually(1, states, optParams.goal{goal_d}.poly, 0:goal_t*numel(dT), C);
    
end

% pairwise distances
if(N_drones>1)
combos = nchoosek(1:N_drones,2);
for p = 1:size(combos,1)
    H_ij = min(DL(combos(p,:))) * N_per_T + 1;
    for k=1:H_ij %for time steps 1 to H_ij, where H_ij=min(H_i, H_j)
        % pa = position of drone a at t=k
        pa = [xx(k,combos(p,1));yy(k,combos(p,1));zz(k,combos(p,1))];
        % pb = position of drone b at t=k
        pb = [xx(k,combos(p,2));yy(k,combos(p,2));zz(k,combos(p,2))];
        % Distance between drone a and drone b - the minimum distance
        mutual_distances(k) = norm(pa-pb,2)-d_min;
    end
    % Find the smallest distance between 2 drones across all timesteps
    dists(p) = SmoothMin(mutual_distances(1:H_ij),C);
end
else
   dists = []; 
end
negative_rob = -SmoothMin([rho_unsafe;rho_goal;dists],C);