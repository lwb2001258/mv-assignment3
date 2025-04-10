function [P1, P2, R, t, R2, t2] = compute_camera_matrices_from_E(E, K1, K2, pts1, pts2)
% Computes P1, P2, R, t from Essential matrix and intrinsics

    % Step 1: Decompose E into R, t
    [U, ~, V] = svd(E);
    if det(U) < 0, U = -U; end
    if det(V) < 0, V = -V; end

    W = [0 -1 0; 1 0 0; 0 0 1];

    R1 = U * W * V';
    R2 = U * W' * V';
    t1 =  U(:,3);
    t2 = -U(:,3);


    % Step 2: Construct P1 and candidates for P2
    P1 = K1 * [eye(3), zeros(3,1)];

    % Four possible configurations of P2
    P2_options = cat(3, ...
        K2 * [R1,  t1], ...
        K2 * [R1,  t2], ...
        K2 * [R2,  t1], ...
        K2 * [R2,  t2]);

    % Step 3: Pick correct P2 by cheirality check
    best_count = 0;
    for i = 1:4
        P2_test = P2_options(:,:,i);
        pts3d = triangulate_points(P1, pts1, P2_test, pts2);

        % Check how many points are in front of both cameras
        X_h = [pts3d, ones(size(pts3d,1),1)]';
        in_front1 = (P1(3,:) * X_h) > 0;
        in_front2 = (P2_test(3,:) * X_h) > 0;
        count = sum(in_front1 & in_front2);

        if count > best_count
            best_count = count;
            P2 = P2_test;
            if i == 1, R = R1; t = t1;
            elseif i == 2, R = R1; t = t2;
            elseif i == 3, R = R2; t = t1;
            elseif i == 4, R = R2; t = t2;
            end
        end
    end
end

% Triangulation helper function
function pts3d = triangulate_points(P1, pts1, P2, pts2)
    N = size(pts1,1);
    pts3d = zeros(N,3);
    for i = 1:N
        A = [pts1(i,1)*P1(3,:) - P1(1,:);
             pts1(i,2)*P1(3,:) - P1(2,:);
             pts2(i,1)*P2(3,:) - P2(1,:);
             pts2(i,2)*P2(3,:) - P2(2,:)];
        [~, ~, V] = svd(A);
        X = V(:,end);
        X = X ./ X(4);
        pts3d(i,:) = X(1:3)';
    end
end
