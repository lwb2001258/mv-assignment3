data=load('data/some_corresp.mat');
im1 = imread('data/im1.png');
M = max(size(im1));
pts1 = double(data.pts1);
pts2 = double(data.pts2);
F_ransac = F_Ransac_Matrix(pts1, pts2, M);
disp('RANSAC Estimated Fundamental Matrix:');
disp(F_ransac);
F=F_ransac;
F = eight_point(pts1, pts2, M);


%% --- Compute Essential Matrix ---
E = compute_essential_matrix(F, K1, K2)


%% --- compute R1,t1,R2,t2,P1,P2 ---
[P1, P2, R1, t1, R2, t2] = compute_camera_matrices_from_E(E, K1, K2, pts1, pts2)
[pts3d, reprojection_error] = triangulate_check(P1, pts1, P2, pts2)
disp(['Mean Reprojection Error: ', num2str(reprojection_error)]);


function [F_best, inliers] = ransac_fundamental(pts1, pts2, M, threshold, iterations)
% Estimate robust fundamental matrix using RANSAC + eight_point()

    N = size(pts1, 1);
    max_inliers = 0;
    F_best = [];
    inliers = [];

    for i = 1:iterations
        % Randomly sample 8 correspondences
        idx = randperm(N, 8);
        F_candidate = eight_point(pts1(idx,:), pts2(idx,:), M);

        % Compute epipolar errors for all points
        errors = compute_epipolar_errors(F_candidate, pts1, pts2);

        % Inliers are those with error below threshold
        inlier_mask = errors < threshold;
        num_inliers = sum(inlier_mask);

        if num_inliers > max_inliers
            max_inliers = num_inliers;
            F_best = F_candidate;
            inliers = inlier_mask;
        end
    end

    % Recompute F using all inliers
    if sum(inliers) >= 8
        F_best = eight_point(pts1(inliers,:), pts2(inliers,:), M);
    end
end
function errors = compute_epipolar_errors(F, pts1, pts2)
    N = size(pts1,1);
    pts1_h = [pts1, ones(N,1)];
    pts2_h = [pts2, ones(N,1)];

    % Epipolar constraint: x2' * F * x1 = 0
    Fx1 = F * pts1_h';
    Ftx2 = F' * pts2_h';

    denom = Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2;
    num = sum((pts2_h * F) .* pts1_h, 2).^2;

    errors = num ./ denom';
end
