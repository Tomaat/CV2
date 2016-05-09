xmean = sum(X,2)./sum(pv_mat,2); 
ymean = sum(Y,2)./sum(pv_mat,2);

Xn = X ./ repmat(xmean,1,size(pv_mat,2));
Yn = Y ./ repmat(ymean,1,size(pv_mat,2));

Xd = Xn(:, sum(pv_mat) == 5);
Yd = Yn(:, sum(pv_mat) == 5);

D = [Xd;Yd];

[U,W,V] = svd(D);

M = U(:,1:3);
S = W(1:3,1:3) * V(:,1:3)';