% ----------- Epipolar Correspondence -----------
function pts2 = epipolar_correspondence(im1, im2, F, pts1)
    if size(im1,3) == 3, im1 = rgb2gray(im1); end
    if size(im2,3) == 3, im2 = rgb2gray(im2); end
    im1 = double(im1);
    im2 = double(im2);
    patch_size = 2;
    search_range = 5;
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