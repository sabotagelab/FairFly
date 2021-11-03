function [DL, failed_idxes] = DL_fail(DL_original, DL_fail)
% DL_original
%   set of DLs, NxD array (D=#drones)
% DL_fail
%   length vector that is infeasible
    DL = zeros(size(DL_original));
    failed_idxes = zeros(size(DL_original));
    row = 1;
    for i=1:size(DL, 1)
        for j=1:size(DL_fail, 1)
            if any(DL_original(i, :) > DL_fail(j, :))
                DL(row, :) = DL_original(i, :);
                row = row + 1;
            else
                failed_idxes(i) = i;
            end
        end
    end
    % remove extra zero rows
    DL = DL(1:row-1, :);
    
    % remove duplicates
    DL = unique(DL, 'row');
    failed_idxes = nonzeros(failed_idxes);
end

