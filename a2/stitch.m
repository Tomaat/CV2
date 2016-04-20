function stitch(img1,img2)
    [i1,f1,d1] = sift_trim(img1);
    [i2,f2,d2] = sift_trim(img2);
    
    [matches, ~] = vl_ubcmatch(d1, d2);
    
    xy1 = f1(1:2,matches(1,:));
    xy2 = f2(1:2,matches(2,:));
    
    [T,in] = RANSAC([xy1;xy2],4,20);
    
    plotlines(i1,xy1,i2,xy2,in);
    
end