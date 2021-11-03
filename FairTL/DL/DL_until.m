% alpha = phi U_[a,b] psi
% DL(alpha) = {max(L_phi + t' - 1, L_psi + t') | a <= t' <= b, L_phi in DL(phi), L_psi in DL(psi)}
function DL = DL_until(DL_phi, DL_psi, a, b)
    DL = zeros(size(DL_phi, 1) * size(DL_psi, 1) * size(a:b, 2), size(DL_phi, 2));
    row = 1;
    for i=a:b
        for j=1:size(DL_phi, 1)
            for k=1:size(DL_psi, 1)
                DL(row, :) = max(DL_phi(j, :) - 1, DL_psi(k, :)) + i;
                row = row + 1;
            end
        end
    end
    
    % remove duplicates
    DL = unique(DL, 'row');
end