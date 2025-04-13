% ---------------------------
% Implement Triangulation and Implement Triangulation
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
E = compute_essential_matrix(F, K1, K2)
%% --- compute R1,t1,R2,t2,P1,P2 ---
[P1, P2, R1, t1, R2, t2] = compute_camera_matrices_from_E(E, K1, K2, pts1, pts2)
[pts3d, reprojection_error] = triangulate_check(P1, pts1, P2, pts2)
disp(['Mean Reprojection Error: ', num2str(reprojection_error)]);


