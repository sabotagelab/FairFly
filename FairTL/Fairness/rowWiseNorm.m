function C = rowWiseNorm(A)
% C(i) = norm of ith row of A
n = size(A,1);
C = zeros(n,1);
for r=1:n
    C(r) = norm(A(r,:));
end
end