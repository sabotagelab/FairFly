function fairness = fairness_weighted(DLs, horizons_min, horizons_max)
    % If h_min, h_max, aren't passed in, then the DLs is the alpha already
    if nargin > 1
        alpha = (DLs - horizons_min)./(horizons_max - horizons_min);
    else
        alpha = DLs;
    end
    
    weights = ones(size(DLs));
    weights(:, 1) = 10;
    weights(:, 2) = 10;

    fairness_weight = 0.75;
    D = size(DLs,2);
    f1 = var(alpha,ones(1,D), 2);
    nn = weighted_norm(alpha, weights);
    fairness = -(fairness_weight * f1) - ((1 - fairness_weight) * nn);    
end


% fairness = zeros(size(DL_set, 1), 1);
% for i=1:size(DL_set, 1)                             % Calculate F
%     fairness(i) = fair_func_two(horizons_min, params.H_drones, DL_set(i, :));
% end

function norm = weighted_norm(alphas, weights)
    norm = zeros(size(alphas, 1), 1);
    for r=1:size(alphas, 1)
        for i=1:length(alphas)
            norm(r) = norm(r) + alphas(r, i)*alphas(r, i)*weights(i);
        end
        norm(r) = norm(r) ^ 0.5;
    end
end