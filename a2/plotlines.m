% Assignment 4 Computer Vision 2016
% author: Ysbrand Galama, 10262067
% Code copied from my previous experience with convolutions:
%   BSc KI cource Beeldverwerken, assignment 4 (2013)
%   and my improvements assisting the same course (2014)

function plotlines(f1,xy,f2,xaya,IN)
    s1 = size(f1);
    s2 = size(f2);
    
    newIm = zeros(max(s1(1),s2(1)),s1(2)+s2(2)+50);
    newIm(1:s1(1),1:s1(2)) = f1;
    newIm(1:s2(1),(end-s2(2)+1):end) = f2;
    xcoords = [xy(1,:);xaya(1,:)+s1(2)+50];
    ycoords = [xy(2,:);xaya(2,:)];
    imshow(newIm)
    hold on
    plot(xcoords(1,:),ycoords(1,:),'.r');
    plot(xcoords(2,:),ycoords(2,:),'.g');
    plot(xcoords,ycoords,'b');
    if nargin > 4
        xcoords = [IN(1,:);IN(3,:)+s1(2)+50];
        ycoords = [IN(2,:);IN(4,:)];
        plot(xcoords(1,:),ycoords(1,:),'.y');
        plot(xcoords(2,:),ycoords(2,:),'.w');
        plot(xcoords,ycoords,'c');
    end
    hold off
end