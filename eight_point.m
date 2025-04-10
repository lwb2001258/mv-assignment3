% ----------- Eight-Point Algorithm -----------
function F = eight_point(pts1, pts2, M)
    T = [1/M, 0, 0; 0, 1/M, 0; 0, 0, 1];
    pts1_h = [pts1, ones(size(pts1,1),1)]';
    pts2_h = [pts2, ones(size(pts2,1),1)]';

    pts1_norm = T * pts1_h;
    pts2_norm = T * pts2_h;

    A = zeros(size(pts1,1),9);
    for i=1:size(pts1,1)
        x1 = pts1_norm(1,i); y1 = pts1_norm(2,i);
        x2 = pts2_norm(1,i); y2 = pts2_norm(2,i);
        A(i,:) = [x1*x2, x1*y2, x1, y1*x2, y1*y2, y1, x2, y2, 1];
    end

    [~, ~, V] = svd(A);
    F_norm = reshape(V(:,end),3,3)';

    [U,S,V] = svd(F_norm);
    S(3,3) = 0;
    F_norm = U*S*V';

    F = T' * F_norm * T;
    F = F / F(3,3);
end