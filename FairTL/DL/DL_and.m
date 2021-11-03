% alpha = phi AND psi
% DL(alpha) = {max(L_phi, L_psi) | L_phi in DL(phi), L_psi in DL(psi)}
function DL = DL_and(DL_phi,DL_psi)
    DL = zeros(size(DL_phi, 1) * size(DL_psi, 1), size(DL_phi, 2));
    row = 1;
    for i=1:size(DL_phi, 1)
        for j=1:size(DL_psi, 1)
            DL(row, :) = max(DL_phi(i, :), DL_psi(j, :));
            row = row + 1;
        end
    end
    
    % remove duplicates
    DL = unique(DL, 'row');
end

