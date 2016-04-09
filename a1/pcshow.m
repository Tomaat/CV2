function pcshow(A,f,A2)
    if nargin < 2
        f = 10;
    end
    
    scatter3(A(1,1:f:end),A(2,1:f:end),A(3,1:f:end) );
    if nargin > 2
        hold on
        scatter3(A2(1,1:f:end),A2(2,1:f:end),A2(3,1:f:end) );
        hold off
    end
end