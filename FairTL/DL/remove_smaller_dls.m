function surviving_search = remove_smaller_dls(DLs, dl, search)
failed_idxes = find_smaller_dls(DLs, dl);
% Remove bad DLs from fairness ordered search space
surviving_search = setdiff(search, failed_idxes);

% U: Lexicographical order -> Fairness order map
% S: Fairness order -> Lexicographical order map
% S = zeros(size(U));
% for i=1:size(U, 1)
%     S(U(i)) = i;
% end
%         % Can't actually lexographically sort, so calculate DLs
%         failed_idxes = find_smaller_dls(DL_set, params.DL);
%         % Remove bad DLs from fairness ordered search space
%         search = setdiff(search, failed_idxes);
%         % Remove bad DLs from lexicographical search space
%         lexi_search = setdiff(lexi_search, S(failed_idxes));