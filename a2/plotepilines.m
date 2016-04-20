function plotepilines(p,f1,f2,T)
    lines = epipolarLine(T,p);
    points = lineToBorderPoints(lines,size(f2));
    
    
    subplot(1,2,1)
    imshow(f1);
    hold on
    plot([p(1,:); p(1,:)],[p(2,:); p(2,:)],'.')
    
    subplot(1,2,2)
    imshow(f2);
    line(points([1 3],:),points([2 4],:));
end