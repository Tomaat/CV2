function [img,f,d] = sift_trim(img,thres)
    if nargin < 2
        thres = 0.09;
    end

    if isscalar(img)
        img = im2double(imread(sprintf('./House/frame%08d.png',img)));
    elseif ischar(img)
        img = im2double(imread(img));
    end
    
    [f,d] = vl_sift(single(img));
    
    [y,x] = find(img<thres);
    xy = [x y]';
    
    elim = false(1,size(f,2));
    for i=1:size(f,2)
        c = round(f(1:2,i));
        elim(i) = any(all(xy == repmat(c,1,size(xy,2))));
    end
    %plot(elim)
    f(:,elim) = [];
    d(:,elim) = [];
end