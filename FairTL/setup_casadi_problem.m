function [solver, lbw, ubw, lbg, ubg] = setup_casadi_problem(params)
    import casadi.*
    
    p0 = params.p0;
    Clen = params.Clen;
    N_drones = params.N_drones;
    DL = params.DL;
    max_per_axis = params.max_per_axis;
    map = params.map;

    % Set up Casadi variables
    p_0 = MX.sym('p_0',3,1);
    w = [];
    lbw = [];
    ubw = [];
    g = [];
    lbg = [];
    ubg = [];


    for d = 1:N_drones
        lbw = [lbw;p0(:,d)];
        ubw = [ubw;p0(:,d)];
        p_0 = MX.sym(['p_' num2str(d) '_' num2str(0)],3,1);
        w = [w;p_0];

        % Only up to each drone's horizon
        for k = 1:DL(d)
            p_k = MX.sym(['p_' num2str(d) '_' num2str(k)],3,1);
            w = [w;p_k];

            % dp for all axes
            dp_x = w(k*3+1+sum(Clen(1:d-1))) - w((k-1)*3+1+sum(Clen(1:d-1)));
            dp_y = w(k*3+2+sum(Clen(1:d-1))) - w((k-1)*3+2+sum(Clen(1:d-1)));
            dp_z = w(k*3+3+sum(Clen(1:d-1))) - w((k-1)*3+3+sum(Clen(1:d-1)));

            % Distance constraints per axis
            g = [g;[dp_x;dp_y;dp_z]];
            lbg = [lbg;-max_per_axis*ones(3,1)];
            ubg = [ubg;+max_per_axis*ones(3,1)];

            % overall bounds on movement
            lbw = [lbw;map.boundary(1:3)'];
            ubw = [ubw;map.boundary(4:6)'];
        end
    end

    % Casadi stuff
    options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 2000, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
        'print_level',0)); %mumps, limited-memory
    options.print_time = false;
    options.expand = false;
    % options.ipopt.print_level = 5;

    prob = struct('f', cost_fair_trajectory_Ndrones(w,params), 'x', w, 'g', g);
    solver = nlpsol('solver', 'ipopt', prob, options);
end

