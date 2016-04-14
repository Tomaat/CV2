function pcshow(A,f)
    if nargin < 2
        f = 13;
    end
    
    
    if size(A,1) == 4
        C = A(4,1:f:end);
    else
        C = zeros(size(A(1,1:f:end)));
    end
    S = ones(size(C)).*1e-99;
    
    scatter3(A(1,1:f:end),A(2,1:f:end),A(3,1:f:end))%,S,C)
    %axis([-.5 .5 .2 1.2 -.5 .5])
end


    %p = 80;
    %p = 342; p=201
    %p=402;%p=171;
    
%     X = reshape(A(1,:),p,[]);
%     Y = reshape(A(2,:),p,[]);
%     Z = reshape(A(3,:),p,[]);
%     
%     if size(A,1) == 4
%         C = reshape(A(4,:),p,[]);
%     else
%         C = zeros(size(X));
%     end
%     
%     surf(X(1:f:end,1:f:end),Z(1:f:end,1:f:end),-Y(1:f:end,1:f:end),C(1:f:end,1:f:end))
%     axis([-1 1 -1 1 -1 1])