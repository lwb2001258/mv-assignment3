% Load stereo images
im1 = imread('data/im1.png');
im2 = imread('data/im2.png');

% Load intrinsics
load('data/intrinsics.mat');  % gives K1, K2

% Load your previously computed extrinsics
load('data/extrinsics.mat');  % gives R1, R2, t1, t2
disp(R1);
disp(t1);
disp(R2);
disp(t2);

% Perform rectification
[img1_rect, img2_rect, M1, M2, K1p, K2p, R1p, R2p, t1p, t2p] = ...
    rectify_stereo_images(im1, im2, K1, K2, R1, R2, t1, t2);

% Display the result
figure;
imshowpair(img1_rect, img2_rect, 'montage');
title('Rectified Temple Image Pair');
