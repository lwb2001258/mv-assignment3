% ---------------------------
% Full Pipeline for Sparse 3D Reconstruction (MATLAB Version) + Epipolar Visualization + Extrinsics Saving
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



%% --- Compute Essential Matrix ---
E = K2' * F * K1;

%% --- calculate P1, P2, R1, t1, R2, t2 from E, K1, K2, pts1, pts2---
[P1, P2, R1, t1, R2, t2] = compute_camera_matrices_from_E(E, K1, K2, pts1, pts2);



%% --- Load temple_coords for 3D reconstruction ---
temple = load('data/temple_coords.mat');
temple_pts1 = double(temple.pts1);
temple_pts2 = epipolar_correspondence(im1, im2, F, temple_pts1);

%% --- Load temple_pts3d from P1, temple_pts1, P2, temple_pts2 ---
[temple_pts3d, reprojection_error] = triangulate_check(P1, temple_pts1, P2, temple_pts2);




%% --- Save Extrinsics ---
save('data/extrinsics.mat','P1','P2','R1','t1','R2','t2');

%% --- Plot 3D points ---
figure; scatter3(temple_pts3d(:,1), temple_pts3d(:,2), temple_pts3d(:,3), 10, 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Reconstructed Temple 3D Point Cloud');
grid on; axis equal;

data = load('data/extrinsics.mat');
disp(data.t1)
