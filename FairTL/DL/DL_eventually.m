% alpha = F_[a,b] psi
% DL(alpha) = {L_psi + t' | a <= t' <= b, L_psi in DL(psi)}
function DL = DL_eventually(DL_psi, a, b)
    DL = zeros(size(DL_psi, 1) * size(a:b, 2), size(DL_psi, 2));
    row = 1;
    % for each drone
    for i = a:b
        for j = 1:size(DL_psi, 1)
            DL(row, :) = DL_psi(j, :) + i;
            row = row + 1;
        end
    end
    
    % remove duplicates
    DL = unique(DL, 'row');
end