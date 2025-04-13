im1 = imread('data/img1_rectified.png');
im2 = imread('data/img2_rectified.png');
% Convert to grayscale if needed
if size(im1,3) == 3, im1 = rgb2gray(im1); end
if size(im2,3) == 3, im2 = rgb2gray(im2); end
% Parameters
max_disp = 50;
win_size = 9;
% Compute disparity
disp_map = compute_disparity_map(im1, im2, max_disp, win_size);
% Display result
figure; imshow(disp_map, []);
title('Disparity Map');
colormap(gca, jet); colorbar;
function disparity_map = compute_disparity_map(im1, im2, max_disp, win_size)
% Compute disparity map from rectified grayscale image pair
% Inputs:
%   im1, im2   - rectified grayscale images (same size)
%   max_disp   - maximum disparity to search (pixels)
%   win_size   - window size (odd integer, e.g., 5, 9)
% Output:
%   disparity_map - disparity value for each pixel

    % Convert to double if needed
    if ~isa(im1, 'double')
        im1 = double(im1);
    end
    if ~isa(im2, 'double')
        im2 = double(im2);
    end
    % Get image size
    [H, W] = size(im1);
    half_win = floor(win_size / 2);
    disparity_map = zeros(H, W);
    % Pad images to handle borders
    im1_pad = padarray(im1, [half_win, half_win], 'replicate');
    im2_pad = padarray(im2, [half_win, half_win + max_disp], 'replicate');
    % Compute disparity using Sum of Squared Differences (SSD)
    for y = 1 + half_win : H - half_win
        for x = 1 + half_win + max_disp : W - half_win
            best_offset = 0;
            min_ssd = inf;
            template = im1_pad(y:y+2*half_win, x:x+2*half_win);
            for d = 0 : max_disp
                block = im2_pad(y:y+2*half_win, x - d:x - d + 2*half_win);
                ssd = sum((template - block).^2, 'all');
                if ssd < min_ssd
                    min_ssd = ssd;
                    best_offset = d;
                end
            end
            disparity_map(y, x) = best_offset;
        end
    end
    % Normalize for display (optional)
    disparity_map = uint8(255 * disparity_map / max_disp);
end
