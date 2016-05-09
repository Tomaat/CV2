[i1,f1,d1] = sift_trim(1);
s(1).i = i1;
s(1).f = f1;
s(1).d = d1;

prev_idx = 1:size(f1,2);
pv_mat = ones(size(prev_idx));
count = 0;

Xt = f1(1,:);
Yt = f1(2,:);
p_ori = f1(1:2,:);

for i=2:15
    i1 = s(i-1).i;
    f1 = s(i-1).f;
    d1 = s(i-1).d;
    
    [i2,f2,d2] = sift_trim(i);
    [matches, ~] = vl_ubcmatch(d1, d2);
    
    xy1 = f1(1:2,matches(1,:));
    xy2 = f2(1:2,matches(2,:));
    
    [T,in,out] = RANSAC([xy1;xy2],4,20);
    
    %figure,
    %subplot(2,1,1)
    %plotlines(i1,xy1,i2,xy2,in);
    %[y,x] = meshgrid(170:170:400,170:170:400); p = [x(:) y(:)]';
    %figure,
    %plotepilines(p,i1,i2,T);
    
    s(i).i = i2;
    s(i).f = f2;
    s(i).d = d2;
    %s(i).xy1 = xy1;
    %s(i).xy2 = xy2;
    s(i).T = T;
    s(i).in = in;
    s(i).out = out;
    
    p_ori_new = zeros(size(p_ori));
    for k=1:size(s(i).in,2)
        match = all( abs(repmat(s(i).in(1:2,k),1,size(p_ori,2)) - p_ori) < 0.1 );
        if any(match)
            % set true
            tmp = find(match);
            pv_mat(i,tmp(1)) = 1;
            Xt(i,tmp(1)) = s(i).in(3,k);
            Yt(i,tmp(1)) = s(i).in(4,k);
            p_ori_new(:,tmp(1)) = s(i).in(3:4,k);
        else
            % add column
            pv_mat(i,end+1) = 1;
            Xt(i,end+1) = s(i).in(3,k);
            Yt(i,end+1) = s(i).in(4,k);
            p_ori_new(:,end+1) = s(i).in(3:4,k);
        end
    end
    for q=1:size(s(i).out,2)
        pv_mat(i,end+1) = 1;
        Xt(i,end+1) = s(i).out(3,q);
        Yt(i,end+1) = s(i).out(4,q);
        p_ori_new(:,end+1) = s(i).out(3:4,q);
    end
    p_ori = p_ori_new;
    
    if i > 4
        X = Xt((i-4):i,:);
        Y = Yt((i-4):i,:);
        pv_mati = pv_mat((i-4):i,:);
        
        xmean = sum(X,2)./sum(pv_mati,2); 
        ymean = sum(Y,2)./sum(pv_mati,2);

        Xn = X ./ repmat(xmean,1,size(pv_mati,2));
        Yn = Y ./ repmat(ymean,1,size(pv_mati,2));

        Xd = Xn(:, sum(pv_mati) == 5);
        Yd = Yn(:, sum(pv_mati) == 5);

        D = [Xd;Yd];

        [U,W,V] = svd(D);

        M = U(:,1:3);
        S = W(1:3,1:3) * V(:,1:3)';
        scatter3(S(1,:),S(2,:),S(3,:));
        hold on
        %pause
    end
    %subplot(2,1,2)
    %imshow(imtransform(pv_mat,makeTform('affine',[1,0,0;0,10,0;0,0,1]),'nearest')); pause;
    %prev_idx = new_prev_idx;
end
zlim([-0.01,0.01])