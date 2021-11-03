function results = solve_fairTL_problem(params, DL_set)
    iter = 1;
    prevFairest = [];
    feasibleFair = [];
    max_iterations = 10000;
    % Solve
    while iter < max_iterations % Loop until explicit break, or exceed maxi  
        % disp(iter);
        %% Search for fairness
        % Try next fairest length vector
        % Use first index of the search list (ordered by fairness)
        params.DL = DL_set.fairest(prevFairest);
        result = solve_fair_trajectory(params);
        result.fairness = DL_set.get_fairness_val(params.DL);
        result.DL = params.DL;
        results{iter} = result;

        %% Remove failures
        if result.negative_rob > 0 || isnan(result.negative_rob)
            DL_set      = DL_set.remove_smaller_dls(params.DL);
            prevFairest = params.DL;
        else
            feasibleFair = result;
            break
        end
        iter = iter + 1;
        
        %% Search for elimination
        % Pick pseudo random DL to eliminate with
        params.DL = DL_set.kinda_random_tuple();
        result = solve_fair_trajectory(params);
        result.fairness = DL_set.get_fairness_val(params.DL);
        result.DL = params.DL;
        results{iter} = result;

        %% Remove failures
        if result.negative_rob > 0 || isnan(result.negative_rob)
            DL_set = DL_set.remove_smaller_dls(params.DL);
        % Or remove stuff with less fairness
        else
            % Remove everything after current because they are all less fair
            feasibleFair = result;
            fvalue       = DL_set.get_fairness_val(params.DL);
            DL_set       = DL_set.remove_less_fair_dls(params.DL, fvalue);
        end
        iter = iter + 1;
    end
    
    results{end} = feasibleFair;
end

