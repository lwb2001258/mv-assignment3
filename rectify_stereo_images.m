function [img1_rect, img2_rect, M1, M2, K1p, K2p, R1p, R2p, t1p, t2p] = ...
         rectify_stereo_images(im1, im2, K1, K2, R1, R2, t1, t2)

    % Step 1: Average intrinsics for new (rectified) intrinsics
    f = (K1(1,1) + K2(1,1)) / 2;
    cx = (K1(1,3) + K2(1,3)) / 2;
    cy = (K1(2,3) + K2(2,3)) / 2;
    K1p = [f, 0, cx; 0, f, cy; 0, 0, 1];
    K2p = K1p;  % use same intrinsics for rectified pair

    % Step 2: Compute new rectified rotation
    t_rel = t2 - t1;             % baseline direction
    z = t_rel / norm(t_rel);     % z-axis: baseline
    x = cross([0; 1; 0], z);     
    x = x / norm(x);             % x-axis orthogonal to z
    y = cross(z, x);             % y-axis
    R_rect = [x'; y'; z'];       % rectified rotation matrix

    % Step 3: New extrinsic parameters
    R1p = R_rect;
    R2p = R_rect;
    t1p = [0; 0; 0];
    t2p = R_rect * t_rel;

    % Step 4: Compute rectification homographies
    M1 = K1p * R1p * R1';
    M2 = K2p * R2p * R2';

    % Step 5: Warp images using projective transformation
    tform1 = projective2d(M1');
    tform2 = projective2d(M2');

    % Determine new output reference size (use original size)
    ref = imref2d(size(im1));  % assumes im1 and im2 are the same size

    img1_rect = imwarp(im1, tform1, 'OutputView', ref);
    img2_rect = imwarp(im2, tform2, 'OutputView', ref);


imwrite(img1_rect, 'results/img1_rectified.png');
imwrite(img2_rect, 'results/img2_rectified.png');
end
