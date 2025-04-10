%% --- Epipolar Lines Visualization ---
figure;
subplot(1,2,1);

imshow(im1); hold on;
title('Epipolar Lines on Image 1');
N = size(pts1,1)
for i=1:10:N
    l = F' * [pts2(i,:), 1]';
    x = 1:size(im1,2);
    y = -(l(1)*x + l(3))/l(2);
    plot(x, y, 'r');
    plot(pts1(i,1), pts1(i,2), 'go');
end
subplot(1,2,2); 
imshow(im2); hold on;
title('Epipolar Lines on Image 2');
for i=1:10:N
    l = F * [pts1(i,:), 1]';
    x = 1:size(im2,2);
    y = -(l(1)*x + l(3))/l(2);
    plot(x, y, 'b');
    plot(pts2(i,1), pts2(i,2), 'go');
end
