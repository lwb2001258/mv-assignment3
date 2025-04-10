dispM = imread('results/disparity_image.jpg');
depthM = compute_depth_map(dispM, K1, K2, R1, R2, t1, t2);
% Visualize depth map
figure;
imagesc(depthM); colormap('jet'); colorbar;
title('Computed Depth Map');
function depthM = compute_depth_map(dispM, K1, K2, R1, R2, t1, t2)
% COMPUTE_DEPTH_MAP computes the depth map from a disparity map and camera parameters
%
% Inputs:
%   dispM - disparity map (2D matrix)
%   K1, K2 - intrinsic matrices for left and right cameras
%   R1, R2 - rotation matrices for left and right cameras
%   t1, t2 - translation vectors for left and right cameras
%
% Output:
%   depthM - computed depth map (same size as dispM)

    % Get focal length from K1 (assumes fx = K1(1,1))
    f = K1(1, 1);

    % Compute camera centers in world coordinates
    C1 = -R1' * t1;
    C2 = -R2' * t2;

    % Compute baseline (distance between optical centers)
    b = norm(C1 - C2);

    % Initialize depth map
    depthM = zeros(size(dispM));

    % Compute depth for each pixel
    valid = dispM > 0;  % mask where disparity is non-zero
    depthM(valid) = (b * f) ./ dispM(valid);
end
