epi_pts2 = epipolar_correspondence(im1, im2, F, pts1);
visualize_point_matches('data/im1.png', 'data/im2.png', pts1, epi_pts2);


function visualize_point_matches(im1_path, im2_path, pts1, epi_pts2)

    % Load images
    im1 = imread(im1_path);
    im2 = imread(im2_path);

    % Resize to same height if needed (optional)
    if size(im1,1) ~= size(im2,1)
        H = max(size(im1,1), size(im2,1));
        im1 = imresize(im1, [H NaN]);
        im2 = imresize(im2, [H NaN]);
    end

    % Concatenate images horizontally
    combined = [im1, im2];

  

    % Offset for image 2 (x direction)
    offset = size(im1,2);
    N = size(pts1, 1);
    % Pick only 20 points to display
    idx = round(linspace(1, size(pts1,1), 20));
    batch_size = 20;
    for start_idx = 1:batch_size:N
        end_idx = min(start_idx + batch_size - 1, N);
        batch_idx = start_idx:end_idx;
          % Display
        figure; imshow(combined); hold on;
        title(sprintf('Visual Comparison of Matched Points %d', idx));
        for i = 1:length(batch_idx)
            p1 = pts1(batch_idx(i), :);
            %p1 = pts1(idx(i), :);
            %p2 = epi_pts2(idx(i), :);
            p2 = epi_pts2(batch_idx(i), :);

            % Random color
            c = rand(1,3);

            % Plot point in image 1
            plot(p1(1), p1(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k');

            % Plot point in image 2 (shifted x)
            plot(p2(1)+offset, p2(2), 's', 'MarkerSize', 8, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k');

            % Draw line between points
            line([p1(1), p2(1)+offset], [p1(2), p2(2)], 'Color', c, 'LineWidth', 1.5);
        end
    end

end

