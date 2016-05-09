[i1,f1,d1] = sift_trim(1);
s(1).i = i1;
s(1).f = f1;
s(1).d = d1;

prev_idx = 1:size(f1,2);
pv_mat = ones(size(prev_idx));
count = 0;

p_ori = f1(1:2,:);

for i=2:5
    i1 = s(i-1).i;
    f1 = s(i-1).f;
    d1 = s(i-1).d;
    
    [i2,f2,d2] = sift_trim(i);
    
    
    [matches, ~] = vl_ubcmatch(d1, d2);
    
    xy1 = f1(1:2,matches(1,:));
    xy2 = f2(1:2,matches(2,:));
    
    [T,in,out] = RANSAC([xy1;xy2],4,20);
    
    %figure,
    subplot(2,1,1)
    plotlines(i1,xy1,i2,xy2,in);
    
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
    
    %p2 = f2(1:2,:);
    p_ori_new = zeros(size(p_ori));
    %new_prev_idx = zeros(1,size(s(i).in,2));
    for k=1:size(s(i).in,2)
        match = all( abs(repmat(s(i).in(1:2,k),1,size(p_ori,2)) - p_ori) < 0.1 );
        if any(match)
            count = count + 1;
            % set true
            tmp = find(match);
            pv_mat(i,tmp(1)) = 1;
            p_ori_new(:,tmp(1)) = s(i).in(3:4,k);
        else
            % add column
            count = count + 0.00001;
            pv_mat(i,end+1) = 1;
            p_ori_new(:,end+1) = s(i).in(3:4,k);
        end
    end
    for q=1:size(s(i).out,2)
        pv_mat(i,end+1) = 1;
        p_ori_new(:,end+1) = s(i).out(3:4,q);
    end
    p_ori = p_ori_new
    subplot(2,1,2)
    imshow(imtransform(pv_mat,makeTform('affine',[1,0,0;0,10,0;0,0,1]),'nearest')); pause;
    prev_idx = new_prev_idx;
end