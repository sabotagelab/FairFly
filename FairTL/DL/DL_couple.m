% alpha(x,y,z,...) = phi(x, y, ...)
% DL(alpha) = {<L_x, L_y, 0, ...> | <L_x, L_y, ...> in DL(phi)}
function DL = DL_couple(DL_phi, num_agents, agents_idx)
    DL = zeros(size(DL_phi, 1), num_agents);
    for i = 1:size(agents_idx, 2)
        DL(:, agents_idx(i)) = DL_phi(:, i);
    end
    
    % remove duplicates
    DL = unique(DL, 'row');
end