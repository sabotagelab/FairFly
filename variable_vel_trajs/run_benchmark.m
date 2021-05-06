test_data.FiveDrones = run_test(5);
%test_data.TenDrones = run_test(10);
%test_data.FifteenDrones = run_test(15);
test_data.TwentyDrones = run_test(20);
test_data.FiftyDrones = run_test(50);

function results = run_test(N_drones)
    disp(["Currently solving for ", N_drones, " drones"]);
    while (1)
        reach_avoid_Ndrones_varvel;
        if (negative_rob < 0)
            break;
        end
        disp("Robustness not positive. Trying again");
    end

    times = ((0:Nsteps)) * H_formula / Nsteps;
    for i=1:N_drones
        results{i} = [times' xx(:, i) yy(:, i) zz(:, i)];
    end
end

