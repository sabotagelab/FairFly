function plot_results(params,results,filename)
    if ~exist('dpg')
        dpg = 1;
    end
    record_gif = false;
    if nargin > 2
        record_gif = true;
    end
    [params.map, params.obs] = plot_env(params.goal, "map0.txt");

    for i=1:params.N_drones
        plot3(params.p0(1, i), params.p0(2, i), params.p0(3, i), 'o', 'Color', params.goal{ceil(i/dpg)}.col);
    end

    N_drones = params.N_drones;
    DL = results.DL;
    params.DL = DL;
    h = params.h;
    params.Clen = 3*(DL+1);
    Nsteps_drones = DL*params.N_per_T;
    drone_colors = params.drone_colors;
    
    [~,xx,yy,zz] = cost_fair_trajectory_Ndrones(results.w_opt,params);

    waypoints = cell(N_drones,1);
    for d = 1:N_drones
        % Plot drones location at each second
        waypoints{d} = reshape(results.w_opt(1+sum(DL(1:d-1)+1)*3:...
            sum(DL(1:d)+1)*3),3,DL(d)+1);
        plot3(waypoints{d}(1,:),waypoints{d}(2,:),waypoints{d}(3,:),'x', 'Color', drone_colors(ceil(d/dpg), :));
        
    end
    
    pause(0.1);
    for d = 1:N_drones
        % Only plot the trajectory for each drone up to its horizon
        plot3(xx(1:Nsteps_drones(d)+1,d),yy(1:Nsteps_drones(d)+1,d),...
            zz(1:Nsteps_drones(d)+1,d), 'Color', drone_colors(ceil(d/dpg), :), 'LineWidth', 1.0);
    end
    
    for t = 1:max(Nsteps_drones) + 1
        gc = zeros(N_drones, 1);
        for d = 1:N_drones
            % Don't plot drone d if passed its horizon
            if t <= Nsteps_drones(d) + 1
                gc(d) = plot3(xx(t,d),yy(t,d),zz(t,d),'d', 'Color', drone_colors(ceil(d/dpg), :), 'MarkerSize',10);
            end
        end
        pause(h); 
        if record_gif
            drawnow
            frame = getframe(1);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im, 256);
            
            if t == 1;
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append');
            end
        end
        delete(gc);
    end
end

