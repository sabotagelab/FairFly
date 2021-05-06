num_iters = 100;


results = cell(30, 1);
for n = 10:2:30
    fprintf("running %d drones\n", n);
    [res.fails, res.successes, res.run_times, res.setup_time] = benchmark(n, num_iters);
    results{n} = res;
end