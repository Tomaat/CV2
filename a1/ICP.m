function [Rout,tout] = ICP(A1,A2,thresh)
    if nargin < 3
        thresh = 0.0012;
    end
    
    Rout = eye(3);
    tout = zeros(3,1);
    
    err = 10;
    [~,W] = size(A1);
    [~,W2] = size(A2);
    iter = 0;
    A2r = A2;
    
    while err > thresh
        iter = iter + 1;
        % phase 1
        idxes = zeros(1,W);
        for i=1:W
            [~,idx] = min( sum( (A2-repmat(A1(:,i),1,W2)).^2 ) );
            idxes(i) = idx;
        end

        % phase 2
        A1c = mean(A1,2);
        A2c = mean(A2,2);
        A1m = A1 - repmat(A1c,1,W);
        A2m = A2 - repmat(A2c,1,W2);

        A2m = A2m(:,idxes);

        % phase 3
        A = zeros(3,3);
        for i=1:W
            A = A + A2m(:,i)*A1m(:,i)';
        end

        [U,~,V] = svd(A);
        
        % phase 4
        R = U*V';
        t = A1c - R*A2c;

        % phase 5
        Rout = R' * Rout;
        tout = tout + t;

        A2 = Rout*A2r + repmat(tout,1,W); % always with original for numeric stability
        err = mean( sum((A1 - A2(:,idxes) ).^2) );
        
        %pcshow(A1,10,A2);
        %pause(0.001);
    end
    disp(iter)
end
