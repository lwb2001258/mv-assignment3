function [img1_rect, img2_rect, M1, M2, K1p, K2p, R1p, R2p, t1p, t2p] = ...
    image_rectification(img1, img2, K1, K2, R1, R2, t1, t2)
%IMAGE_RECTIFICATION Computes stereo rectification for a pair of calibrated cameras
% 1. Compute projection matrices
P1 = K1 * [R1, t1];
P2 = K2 * [R2, t2];
% 2. Compute the new stereo baseline
% Center of each camera
c1 = -R1' * t1;
c2 = -R2' * t2;
baseline = c2 - c1;
% 3. Compute the new rectified camera coordinate system
z = baseline / norm(baseline);
v1 = R1(3,:)';  % optical axis of camera 1
x = cross(v1, z); x = x / norm(x);
y = cross(z, x);
% 4. Form new rotation matrix
R_rect = [x'; y'; z'];
% 5. New intrinsic matrices (assume unchanged or average)
K1p = K1;
K2p = K2;
% 6. Compute new extrinsic matrices
R1p = R_rect;
R2p = R_rect;
t1p = -R1p * c1;
t2p = -R2p * c2;
% 7. Compute new projection matrices
P1p = K1p * [R1p, t1p];
P2p = K2p * [R2p, t2p];
% 8. Compute rectification homographies (Hartley method)
H1 = P1p(:,1:3) / P1(:,1:3);
H2 = P2p(:,1:3) / P2(:,1:3);
M1 = H1;  % rectification matrix for image 1
M2 = H2;  % rectification matrix for image 2
% 9. Warp images
tform1 = projective2d(M1');
tform2 = projective2d(M2');
img1_rect = imwarp(img1, tform1, 'OutputView', imref2d(size(img1)));
img2_rect = imwarp(img2, tform2, 'OutputView', imref2d(size(img2)));
end
