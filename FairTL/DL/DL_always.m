% alpha = G_[a,b] psi
% DL(alpha) = {L_psi + b | L_psi in DL(psi)}
function DL = DL_always(DL_psi, a, b)
    DL = DL_psi + b;
    
    % remove duplicates
    DL = unique(DL, 'row');
end