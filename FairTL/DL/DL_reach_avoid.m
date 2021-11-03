% alpha = phi AND psi
% DL(alpha) = {max(L_phi, L_psi) | L_phi in DL(phi), L_psi in DL(psi)}
function DL = DL_reach_avoid(horizon)
    DL = zeros(size(horizon));
    for i=1:size(horizon, 2)
        % goal
        DL = DL_and(DL, DL_couple(DL_eventually(1, 0, horizon(i)), size(horizon, 2), i));
        % obstacle
        %DL = DL_and(DL, DL_couple(DL_always(1, 0, horizon(i)), size(horizon, 2), i));
        % mutual avoidance
        %for j=1:size(horizon, 2)
        %    if i == j
        %        continue
        %    end
        %    DL = DL_and(DL, DL_couple(ones(1,2) * min(horizon(i), horizon(j)), size(horizon, 2), [i, j]));
        %end
    end
    
    % remove duplicates
    DL = unique(DL, 'row');
end

