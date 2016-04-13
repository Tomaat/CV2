function pcshowpair(A,B,f)
    if nargin < 3
        f = 23;
    end
    
    pcshow(A,f)
    hold on
    pcshow(B,f)
    hold off
end