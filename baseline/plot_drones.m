function plot_drones(X, W, obst, boundary)
figure;
colors = distinguishable_colors(size(W,1),{'w', 'r'},@(x) colorspace('RGB->Lab',x));
plot(Polyhedron('lb', obst(1,:), 'ub', obst(2,:)), 'Color', 'red', 'alpha', 0.5)
hold on;
for d = 1:size(W,1)
    x = squeeze(X(d,:,:));
    w = squeeze(W(d,:,:));
    plot3(w(1,1), w(1,2), w(1,3), 'o', 'Color', colors(d,:));
    xlim(boundary(:,1));
    ylim(boundary(:,2));
    zlim(boundary(:,3));
    plot3(w(end,1), w(end,2), w(end,3), 's', 'Color', colors(d,:));
    
    %plot3(w(:,1), w(:,2), w(:,3), strcat(colors(d), '+'));
    plot3(x(:,1), x(:,2), x(:,3), '-', 'Color', colors(d,:));
end

for t = 1:size(X,2)
    if(exist('gc','var'))
        delete(gc)
    end
    for d = 1:size(X,1)
        x = squeeze(X(d,t,:));
        gc(d) = plot3(x(1), x(2), x(3), 'x', 'Color', colors(d,:));
    end
    pause(0.1);
end

end

