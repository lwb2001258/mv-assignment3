function [pts3d, reprojection_error] = triangulate_check(P1, pts1, P2, pts2)
% Triangulate 3D points and compute mean reprojection error
% Inputs:
%   P1, P2: 3x4 projection matrices
%   pts1, pts2: Nx2 matching points
% Outputs:
%   pts3d: Nx3 triangulated 3D points
%   reprojection_error: mean reprojection error across both views

    N = size(pts1, 1);
    pts3d = zeros(N, 3);
    err1 = zeros(N, 1);
    err2 = zeros(N, 1);

    for i = 1:N
        A = [pts1(i,1)*P1(3,:) - P1(1,:);
             pts1(i,2)*P1(3,:) - P1(2,:);
             pts2(i,1)*P2(3,:) - P2(1,:);
             pts2(i,2)*P2(3,:) - P2(2,:)];

        [~, ~, V] = svd(A);
        X = V(:, end);
        X = X ./ X(4);

        pts3d(i, :) = X(1:3)';

        % Reproject
        x1_proj = P1 * X;
        x1_proj = x1_proj(1:2) / x1_proj(3);

        x2_proj = P2 * X;
        x2_proj = x2_proj(1:2) / x2_proj(3);

        % Compute per-point reprojection errors
        err1(i) = norm(x1_proj' - pts1(i,:));
        err2(i) = norm(x2_proj' - pts2(i,:));
    end

    % Average reprojection error over all points
    reprojection_error = mean(err1 + err2);
end
