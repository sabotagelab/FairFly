function fairness = fairness1(DLs, horizons_min, horizons_max)
    % If h_min, h_max, aren't passed in, then the DLs is the alpha already
    if nargin > 1
        alpha = (DLs - horizons_min)./(horizons_max - horizons_min);
    else
        alpha = DLs;
    end
    
    D = size(DLs,2);
    fairness = -var(alpha,ones(1,D), 2);
end