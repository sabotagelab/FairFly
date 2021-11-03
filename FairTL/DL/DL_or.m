% alpha = phi OR psi
% DL(alpha) = {L_phi, L_psi | L_phi in DL(phi), L_psi in DL(psi)}
function DL = DL_or(DL_phi,DL_psi)
    DL = [DL_phi; DL_psi];
    
    % remove duplicates
    DL = unique(DL, 'row');
end