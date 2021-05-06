function [p0] = random_p0_generator(map,obs,N_drones, dx)
if nargin < 4
    dx = 1.5;
end

% region
Region = Polyhedron('lb',map.boundary(1:3),'ub',map.boundary(4:6));

% Initial free space
FreeSpace = Region;
for i = 1:size(obs)
    FreeSpace =  FreeSpace\obs{i}.shape;
end

p0 = zeros(3,N_drones);

for i = 1:N_drones
    % Select random  center point from freespace
    j = randi(size(FreeSpace,1),1);
    
    % Assign random initial position
    p0(:,i) = FreeSpace(j).Internal.InternalPoint.x;
    
    % Treat Assigned point as Obstacle, Recalculate Freespace
    tempObs = Polyhedron('lb',p0(:,i)-dx,'ub',p0(:,i)+dx);
    FreeSpace =  FreeSpace\tempObs;
    
    new_size = size(FreeSpace, 1);
end

if(0)
    plot(FreeSpace);
end


