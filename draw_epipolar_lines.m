%% --- Epipolar Lines Visualization ---
figure;
N = size(pts1, 1);
% --- Image 1: Epipolar lines from pts2 to im1 ---
subplot(1,2,1);
imshow(im1); hold on;
title('Epipolar Lines on Image 1');
for i = 1:10:N
    % Compute epipolar line in image 1 from pts2
    l = F' * [pts2(i,:), 1]';
    x = 1:size(im1,2);
    y = -(l(1)*x + l(3)) / l(2);    
    % Generate random RGB color
    c = rand(1,3);    
    % Plot line and corresponding point in same color
    plot(x, y, 'Color', c, 'LineWidth', 1.2);
    plot(pts1(i,1), pts1(i,2), '.', 'Color', c, 'MarkerSize', 15);
end
% --- Image 2: Epipolar lines from pts1 to im2 ---
subplot(1,2,2);
imshow(im2); hold on;
title('Epipolar Lines on Image 2');
for i = 1:10:N
    % Compute epipolar line in image 2 from pts1
    l = F * [pts1(i,:), 1]';
    x = 1:size(im2,2);
    y = -(l(1)*x + l(3)) / l(2);
    % Generate random RGB color
    c = rand(1,3);    
    % Plot line and corresponding point in same color
    plot(x, y, 'Color', c, 'LineWidth', 1.2);
    plot(pts2(i,1), pts2(i,2), '.', 'Color', c, 'MarkerSize', 15);
end
