function failed_indices = find_smaller_dls(DLs, dl)
% failed_indices
%   indices of rows of DLs that are smaller (elt-wise) than dl
% DL_original
%   set of DLs, NxD array (D=#drones)
% dl
%   length vector that is infeasible


failed_indices = find(all(DLs <= dl, 2));


