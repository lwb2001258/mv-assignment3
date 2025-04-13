function E = compute_essential_matrix(F, K1, K2)
% Compute Essential Matrix from Fundamental Matrix and Intrinsics
    E = K2' * F * K1;
end
