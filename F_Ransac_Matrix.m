function Best_F_matrix = F_Ransac_Matrix(pts1,pts2,M)
    err_thresh = 0.02;
    inliers_thresh = 0;
    Best_F_matrix = [];
    selected_indices = [];

    n_rows = size(pts1,1);

    for i = 1:1000
        % Randomly select 8 unique rows (indices)
        random_pairs = randperm(n_rows, 8);
        pts1_s = pts1(random_pairs,:);
        pts2_s = pts2(random_pairs,:);

        % Estimate F from 8 correspondences
        F = eight_point(pts1_s,pts2_s,M);

        indices = [];
        for j = 1:n_rows
            
            pts1_e = pts1(j,:);
            pts2_e = pts2(j,:);
            error = fundamental_error(pts1_e,pts2_e, F);

            if error < err_thresh
                indices(end + 1) = j;
            end
        end

        if length(indices) > inliers_thresh
            inliers_thresh = length(indices);
            selected_indices = indices;
            Best_F_matrix = F;
        end
    end


    disp('Best Fundamental Matrix:');
    disp(Best_F_matrix);
end

function error = fundamental_error(pts1_e,pts2_e, F)
   
    

    % Convert to homogeneous coordinates
    disp(pts1_e);
    pts1_h = [pts1_e, 1]';  % 3x1 column
    pts2_h = [pts2_e, 1]';  % 3x1 column

    % Compute epipolar constraint: x2' * F * x1
    error = abs(pts2_h' * F * pts1_h);
end
