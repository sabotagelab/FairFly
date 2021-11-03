function fairness = fair_func_one_weighted(horizons_min, horizons_max, DLs, weights)
    alpha = (DLs - horizons_min)./(horizons_max - horizons_min);
    fairness = -var(alpha, weights);
end