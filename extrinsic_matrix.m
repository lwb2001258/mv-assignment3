
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

[R1, t1, R2, t2] = decompose_essential_matrix(E)

[img1_rect, img2_rect] = rectify_images_from_extrinsics(im1, im2, R1, R2, t1, t2)


function [R1, t1, R2, t2] = decompose_essential_matrix(E)
    % This function decomposes the Essential matrix E into two possible
    % rotation matrices (R1, R2) and two translation directions (t1, t2)
    % such that E = [t]_x * R

    % Perform SVD of E
    [U, ~, V] = svd(E);

    % Ensure U and V are proper rotation matrices
    if det(U) < 0, U = -U; end
    if det(V) < 0, V = -V; end
    

    % W matrix as defined in Hartley & Zisserman
    W = [0 -1 0;
         1  0 0;
         0  0 1];

    % Compute the two possible rotations
    R1 = U * W * V';
    R2 = U * W' * V';

    % Ensure R1 and R2 are proper rotation matrices
    if det(R1) < 0, R1 = -R1; end
    if det(R2) < 0, R2 = -R2; end

    % Possible translations (unit vector)
    t1 = U(:,3);
    t2 = -U(:,3);
end

function [img1_rect, img2_rect] = rectify_images_from_extrinsics(im1, im2, R1, R2, t1, t2)
    % Ensure translation vectors are column vectors
    t1 = t1(:);
    t2 = t2(:);

    % Step 1: Compute relative baseline direction
    baseline = t2 - t1;
    z = baseline / norm(baseline);  % new Z-axis
    
    % Step 2: Define a new rectified coordinate frame (R_rect)
    x = cross([0; 1; 0], z);         % new X-axis
    if norm(x) < 1e-6
        x = cross([1; 0; 0], z);     % fallback if baseline || Y
    end
    x = x / norm(x);
    y = cross(z, x);                 % new Y-axis
    R_rect = [x'; y'; z'];

    % Step 3: Compute rectification homographies
    % Assume identity intrinsic matrix (or use identity rectification model)
    H1 = R_rect * R1';
    H2 = R_rect * R2';

    % Step 4: Apply projective transforms
    tform1 = projective2d(H1');
    tform2 = projective2d(H2');

    % Step 5: Warp images using automatic size
    img1_rect = imwarp(im1, tform1, 'FillValues', 0);
    img2_rect = imwarp(im2, tform2, 'FillValues', 0);

    % Step 6: Resize to same height if needed
    h = max(size(img1_rect, 1), size(img2_rect, 1));
    img1_rect = imresize(img1_rect, [h NaN]);
    img2_rect = imresize(img2_rect, [h NaN]);
end
