function [fair, rob, time, online] = process_results(results_file)
if nargin == 0
    plot_all
    return
end
addpath('Fairness');
close all;
load(results_file);
num_results = length(results);

fairnessFbL = zeros(2, num_results); % Fairness of FbL solution with each F
fairness_1  = zeros(2, num_results);
fairness_2  = zeros(2, num_results);
robustness  = zeros(3, num_results);

fairness2_iters = cell(1, num_results);
fairness2_times = cell(1, num_results);
solve_times = zeros(3, num_results);
online_times = zeros(3, num_results);

for i=1:num_results
    result = results{i};
    %% Grab results from FbL
    robustness(1, i) = -result.FbL.negative_rob;
    solve_times(1, i) = sum_time(result.FbL.times);
    online_times(1, i) = sum_time(result.FbL.times);
    fairnessFbL(1, i) = fairness1(result.FbL.DL, 1, result.params.H_drones);
    fairnessFbL(2, i) = fairness2(result.FbL.DL, 1, result.params.H_drones);
    
    %% Grab results from Fairness function 1
    robustness(2, i) = -result.FairFly1{end}.negative_rob;
    fairness_1(1, i) = fairness1(result.FairFly1{end}.DL, 1, result.params.H_drones);
    fairness_1(2, i) = fairness2(result.FairFly1{end}.DL, 1, result.params.H_drones);
    solve_times(2, i) = sum_time(result.FairFly1{end}.times);
    online_times(2, i) = sum_time(result.FairFly1{end}.times);
    
    %% Grab results from Fairness function 2
    robustness(3, i) = -result.FairFly2{end}.negative_rob;
    fairness_2(1, i) = fairness1(result.FairFly2{end}.DL, 1, result.params.H_drones);
    fairness_2(2, i) = fairness2(result.FairFly2{end}.DL, 1, result.params.H_drones);
    
%    f2_i_iters = zeros(1, length(result.FairFly2_exact));
%    f2_i_times = zeros(1, length(result.FairFly2_exact));
%    for j=1:length(result.FairFly2)
%        f2_i_iters(j) = result.FairFly2{j}.fairness;
%        f2_i_times(j) = sum_time(result.FairFly2_exact{j}.times);
%    end
%    fairness2_iters{i} = f2_i_iters;
%    fairness2_times{i} = f2_i_times;
%    solve_times(3, i) = sum(f2_i_times);
    solve_times(3, i) = result.FairFly2_solve_time;
    online_times(3, i) = sum_time(result.FairFly2{end}.times);
end


%% Plot series of just N drones
if 0
%% Plot results
figure()
hold on;
plot(robustness(1, :), 'k-');
plot(robustness(2, :), 'k--');
%plot(robustness(3, :), 'k-.');
xlabel('Iteration of different initial position');
ylabel('Robustness of result');
%legend({'FbL', 'Fairness 1', 'Fairness 2'}, 'Location', 'southeast');
legend({'FbL', 'FairFly'}, 'Location', 'southeast');
title("Robustness over different initial positions");

figure()
hold on;
plot(solve_times(1, :), 'k-');
plot(solve_times(2, :), 'k--');
%plot(solve_times(3, :), 'k-.');
xlabel('Iteration of different initial position');
ylabel('Time to Solve offline');
%legend({'FbL', 'Fairness 1', 'Fairness 2'}, 'Location', 'northeast');
legend({'FbL', 'FairFly'}, 'Location', 'northeast');
title("Offline time to solve over different initial positions");


figure()
hold on;
plot(fairnessFbL(1, :), 'k-');
plot(fairness_1(1, :), 'k--');
xlabel('Iteration of different initial position');
ylabel('Fairness1 value');
legend({'FbL', 'FairFly1'}, 'Location', 'northeast');
title("Fairest1 solution over different initial positions");

%figure()
%hold on;
%plot(fairnessFbL(2, :), 'k-');
%plot(fairness_2(1, :), 'k--');
%xlabel('Iteration of different initial position');
%ylabel('Fairness2 value');
%legend({'FbL', 'FairFly2'}, 'Location', 'northeast');
%title("Fairest2 solution over different initial positions");

hold on;
end
fair = [mean(fairnessFbL(1, :)); mean(fairness_1(1, :)); mean(fairness_2(1, :)); 
        mean(fairnessFbL(2, :)); mean(fairness_1(2, :)); mean(fairness_2(2, :))];
rob  = [mean(robustness(1, :)); mean(robustness(2, :)); mean(robustness(3, :))];
time = [mean(solve_times(1, :)); mean(solve_times(2, :)); mean(solve_times(3, :))];
online = [mean(online_times(1, :)); mean(online_times(2, :)); mean(online_times(3, :))];

end


function t = sum_time(time_struct)
    t = time_struct.casadi + time_struct.init_sol + time_struct.solve;
end

function plot_all
fair = zeros(6, 4);
rob = zeros(3, 4);
offline = zeros(3, 4);
online = zeros(3, 4);

[fair(:, 1), rob(:, 1), offline(:,1), online(:,1)] = process_results('Results/5d_old.mat');
[fair(:, 2), rob(:, 2), offline(:,2), online(:,2)] = process_results('Results/10d.mat');
[fair(:, 3), rob(:, 3), offline(:,3), online(:,3)] = process_results('Results/15d.mat');
[fair(:, 4), rob(:, 4), offline(:,4), online(:,4)] = process_results('Results/20d.mat');

x_axis = [5 10 15 20];

figure(1)
hold on
plot(x_axis, fair(1, :), 'k-');
plot(x_axis, fair(2, :), 'k-.');
legend({'Fly-by-Logic', 'FairFly1'}, 'Location', 'southeast');
xlim([0,25]);
ylim([-0.25,0.05]);
xlabel('N drones');
ylabel('Average fairness (f1)');
title('Average fairness (f1) for N drones');
%export_fig(["fairness_plot",'.pdf'], '-pdf','-transparent');

figure(2)
hold on
plot(x_axis, fair(4, :), 'k-');
plot(x_axis, fair(6, :), 'k--');
legend({'Fly-by-Logic', 'FairFly2'}, 'Location', 'southeast');
xlim([0,25]);
ylim([-2.5,0]);
xlabel('N drones');
ylabel('Average fairness (f2)');
title('Average fairness (f2) for N drones');
%export_fig(["fairness_plot",'.pdf'], '-pdf','-transparent');

figure(3)
hold on
plot(x_axis, rob(1, :), 'k-');
plot(x_axis, rob(2, :), 'k-.');
plot(x_axis, rob(3, :), 'k--');
legend({'Fly-by-Logic', 'FairFly1', 'FairFly2'}, 'Location', 'southeast');
xlim([0,25]);
ylim([0,0.25]);
xlabel('N drones');
ylabel('Average robustness');
title('Average robustness for N drones');
%export_fig(["robustness_plot",'.pdf'], '-pdf','-transparent');

figure(4)
hold on
plot(x_axis, offline(1, :), 'k-')
plot(x_axis, offline(2, :), 'k-.')
plot(x_axis, offline(3, :), 'k--')
title('Average offline solve time for N drones')
xlabel('N drones')
ylabel('Average solve time (s)')
legend({'Fly-by-Logic', 'FairFly1', 'FairFly2'}, 'Location', 'southeast')
xlim([0,25])
%export_fig(["solve_time_plot",'.pdf'], '-pdf','-transparent');

figure(5)
hold on
plot(x_axis, online(1, :), 'k-')
plot(x_axis, online(2, :), 'k-.')
plot(x_axis, online(3, :), 'k--')
title('Average online solve time for N drones')
xlabel('N drones')
ylabel('Average solve time (s)')
legend({'Fly-by-Logic', 'FairFly1', 'FairFly2'}, 'Location', 'southeast')
xlim([0,25])
%export_fig(["solve_time_plot",'.pdf'], '-pdf','-transparent');

figure(6)
hold on
offline_perc = (offline - offline(1, :)) ./ offline(1, :) * 100;
plot(x_axis, offline_perc(1, :), 'k-')
plot(x_axis, offline_perc(2, :), 'k-.')
plot(x_axis, offline_perc(3, :), 'k--')
title('Average offline solve time for N drones')
xlabel('N drones')
ylabel('% difference of solve time compared to Fly-by-Logic')
legend({'Fly-by-Logic', 'FairFly1', 'FairFly2'}, 'Location', 'southeast')
xlim([0,25])
ylim([-100, 100])
%export_fig(["solve_time_plot",'.pdf'], '-pdf','-transparent');

figure(7)
hold on
online_perc = (online - online(1, :)) ./ online(1, :) * 100;
plot(x_axis, online_perc(1, :), 'k-')
plot(x_axis, online_perc(2, :), 'k-.')
plot(x_axis, online_perc(3, :), 'k--')
title('Average online solve time for N drones')
xlabel('N drones')
ylabel('% difference of solve time compared to Fly-by-Logic')
legend({'Fly-by-Logic', 'FairFly1', 'FairFly2'}, 'Location', 'southeast')
xlim([0,25])
ylim([-100, 100])
%export_fig(["solve_time_plot",'.pdf'], '-pdf','-transparent');

end
