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