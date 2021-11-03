% alpha = phi R_[a,b] psi
% DL(alpha_1) = {t' + L_phi | L_phi in DL(phi), 0 <= t' < a}
% DL(alpha_2) = {max(L_phi + t'', L_psi + t' - 1) | a <= t'' <= t' < b, L_phi in DL(phi), L_psi in DL(psi)}
% DL(alpha_3) = {b + L_psi | L_psi in DL(psi)}
% DL(alpha) = DL(alpha_1) union DL(alpha_2) union DL(alpha_3)
function DL = DL_release(DL_phi, DL_psi, a, b)
    DL_1 = zeros(size(DL_phi, 1) * size(0:a-1, 2), size(DL_phi, 2));
    row = 1;
    for i = 0:a-1
        for j = 1:size(DL_phi, 1)
            DL_1(row, :) = DL_phi(j, :) + i;
            row = row + 1;
        end
    end
    
    DL_2 = zeros(size(DL_phi, 1) * size(DL_psi, 1) * sum(1:b-a), size(DL_phi, 2));
    row = 1;
    for i=a:b-1
        for j=a:i
            for k=1:size(DL_phi, 1)
                for l=1:size(DL_psi, 1)
                    DL_2(row, :) = max(DL_phi(k, :) + j, DL_psi(l, :) + i);
                    row = row + 1;
                end
            end
        end
    end
    
    DL_3 = DL_psi + b;
    
    DL = [DL_1; DL_2; DL_3];
    
    % remove duplicates
    DL = unique(DL, 'row');
end