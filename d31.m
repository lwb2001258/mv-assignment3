% ---------------------------
% Full Pipeline for Sparse 3D Reconstruction (MATLAB Version)
% ---------------------------

clear; clc;

%% --- Load Data ---
data = load('data/some_corresp.mat');
pts1 = double(data.pts1);
pts2 = double(data.pts2);

intrinsics = load('data/intrinsics.mat');
K1 = intrinsics.K1;
K2 = intrinsics.K2;

im1 = imread('data/im1.png');
im2 = imread('data/im2.png');
M = max([size(im1,1), size(im1,2), size(im2,1), size(im2,2)]);

%% --- Compute Fundamental Matrix ---
F = eight_point(pts1, pts2, M);
disp('Estimated Fundamental Matrix F:');
disp(F);

%% --- Compute Essential Matrix ---
E = K2' * F * K1;
disp('Estimated Essential Matrix E:');
disp(E);

%% --- Decompose E to get candidate R, t ---
[U, ~, V] = svd(E);
if det(U)<0, U = -U; end
if det(V)<0, V = -V; end

W = [0 -1 0; 1 0 0; 0 0 1];
R1 = U * W * V';
R2 = U * W' * V';
t = U(:,3);

% Fix det
if det(R1)<0, R1 = -R1; t = -t; end
if det(R2)<0, R2 = -R2; end

%% --- P1 and P2 Candidates ---
P1 = K1 * [eye(3), zeros(3,1)];
P2s = cat(3, K2 * [R1,  t], K2 * [R1, -t], K2 * [R2,  t], K2 * [R2, -t]);

%% --- Load temple_coords for 3D reconstruction ---
temple = load('data/temple_coords.mat');
temple_pts1 = double(temple.pts1);
temple_pts2 = epipolar_correspondence(im1, im2, F, temple_pts1);

%% --- Select Correct P2 via Cheirality Check ---
best_count = 0;
for i = 1:4
    P2 = P2s(:,:,i);
    [pts3d, count] = triangulate_check(P1, temple_pts1, P2, temple_pts2);
    if count > best_count
        best_count = count;
        best_P2 = P2;
        best_pts3d = pts3d;
    end
end

disp('Selected Correct P2:');
disp(best_P2);

%% --- Plot 3D points ---
figure; scatter3(best_pts3d(:,1), best_pts3d(:,2), best_pts3d(:,3), 10, 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Reconstructed Temple 3D Point Cloud');
grid on; axis equal;


% ----------- Eight-Point Algorithm -----------
function F = eight_point(pts1, pts2, M)
    T = [1/M, 0, 0; 0, 1/M, 0; 0, 0, 1];
    pts1_h = [pts1, ones(size(pts1,1),1)]';
    pts2_h = [pts2, ones(size(pts2,1),1)]';

    pts1_norm = T * pts1_h;
    pts2_norm = T * pts2_h;

    A = zeros(size(pts1,1),9);
    for i=1:size(pts1,1)
        x1 = pts1_norm(1,i); y1 = pts1_norm(2,i);
        x2 = pts2_norm(1,i); y2 = pts2_norm(2,i);
        A(i,:) = [x1*x2, x1*y2, x1, y1*x2, y1*y2, y1, x2, y2, 1];
    end

    [~, ~, V] = svd(A);
    F_norm = reshape(V(:,end),3,3)';

    [U,S,V] = svd(F_norm);
    S(3,3) = 0;
    F_norm = U*S*V';

    F = T' * F_norm * T;
    F = F / F(3,3);
end

% ----------- Triangulation + Cheirality Check -----------
function [pts3d, count] = triangulate_check(P1, pts1, P2, pts2)
    N = size(pts1,1);
    pts3d = zeros(N,3);
    count = 0;
    for i=1:N
        A = [pts1(i,1)*P1(3,:) - P1(1,:);
             pts1(i,2)*P1(3,:) - P1(2,:);
             pts2(i,1)*P2(3,:) - P2(1,:);
             pts2(i,2)*P2(3,:) - P2(2,:)];
        [~, ~, V] = svd(A);
        X = V(:,end);
        X = X / X(4);
        pts3d(i,:) = X(1:3)';

        % Check depth > 0
        d1 = P1(3,:) * X;
        d2 = P2(3,:) * X;
        if d1>0 && d2>0
            count = count + 1;
        end
    end
end

% ----------- Epipolar Correspondence -----------
function pts2 = epipolar_correspondence(im1, im2, F, pts1)
    if size(im1,3) == 3, im1 = rgb2gray(im1); end
    if size(im2,3) == 3, im2 = rgb2gray(im2); end

    im1 = double(im1);
    im2 = double(im2);

    patch_size = 5;
    search_range = 30;

    N = size(pts1,1);
    pts2 = zeros(N,2);

    for i=1:N
        x1 = pts1(i,1);
        y1 = pts1(i,2);

        l2 = F * [x1; y1; 1];
        l2 = l2 / sqrt(l2(1)^2 + l2(2)^2);

        ref_patch = get_patch(im1, x1, y1, patch_size);

        min_ssd = inf;
        best_x2 = -1; best_y2 = -1;

        for dy = -search_range:search_range
            x2 = round(x1);
            y2 = round(-(l2(1)*x2 + l2(3)) / l2(2) + dy);

            if y2 <= patch_size || y2 >= size(im2,1) - patch_size || ...
               x2 <= patch_size || x2 >= size(im2,2) - patch_size
                continue;
            end

            target_patch = get_patch(im2, x2, y2, patch_size);
            ssd = sum((ref_patch(:) - target_patch(:)).^2);

            if ssd < min_ssd
                min_ssd = ssd;
                best_x2 = x2;
                best_y2 = y2;
            end
        end

        pts2(i,:) = [best_x2, best_y2];
    end
end

function patch = get_patch(img, x, y, patch_size)
    x = round(x);
    y = round(y);
    patch = img(y-patch_size:y+patch_size, x-patch_size:x+patch_size);
end
