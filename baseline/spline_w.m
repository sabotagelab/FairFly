function x = spline_w(w, v, T, M1, dv, da)
%SPLINE_W converts waypoints [w,v] into a spline x
%   Detailed explanation goes here

N_per_T = 10;
dT = T/N_per_T:T/N_per_T:T;
[N,H,D] = size(w);
H = H-1;

x = zeros(N,H*N_per_T+1,D);
for n = 1:N
    x(n,1,:) = w(n,1,:);
    for t = 1:H
        for d = 1:D
            dp_d = w(n,t+1,d) - w(n,t,d) - T*v(n,t,d);
            al_d = M1(1,:)*[dp_d;dv;da];
            be_d = M1(2,:)*[dp_d;dv;da];
            gam_d = M1(3,:)*[dp_d;dv;da];
            x(n,2+(t-1)*N_per_T:t*N_per_T+1,d) = al_d/120*dT.^5 + ...
                                                 be_d/24*dT.^4 + ...
                                                 gam_d/6*dT.^3 + ...
                                                 w(n,t,d) + dT*v(n,t,d);
        end
    end
end
    
end

