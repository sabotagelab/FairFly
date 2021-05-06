function [start, goal, obst, boundary] = generate_config(N_drones, min_distance)
    obst = [-0.1 -0.1 -0.1; 0.1 0.1 0.1];
    boundary = [-0.5 -0.5 -0.5; 0.5 0.5 0.5];
    
    start = zeros(N_drones, 3);
    goal = zeros(N_drones, 3);
    
    for i = 1:N_drones
        while 1
            % Need a way to track when we know that the points are not
            % interfering with previous points
            stop = true;
            
            face = randi([1 6]);        % Start on a random face
            start(i,:) = rand(1,3)-0.5; % Create random start vector
            goal(i, :) = rand(1,3)-0.5; % Create random goal vector
            
            % Convert the face chosen on start and goal to -0.5/+0.5
            start(i,mod(face, 3) + 1) =  ceil(face/3)-1.5;
            goal(i, mod(face, 3) + 1) = -ceil(face/3)+1.5;
        
            for j = 1:i-1
                % Check if start coord within min_distance of others
                if sum(abs(start(i,:) - start(j,:)) < min_distance) == 3
                    stop = false;
                    break           % Skip and try new vectors
                end
                
                % Check if goal coord within min_distance of others
                if sum(abs(goal(i,:) - goal(j,:)) < min_distance) == 3
                    stop = false;
                    break           % Skip and try new vectors
                end
            end
            
            if stop
                break
            end
        end
    end
end

