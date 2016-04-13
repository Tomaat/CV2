function [Rout,tout] = ICP(source,target,method,thresh)
    if nargin < 4
        thresh = 0.0012;
        if nargin < 3
            method = 'all';
        end
    end
    
    Rout = eye(3);
    tout = zeros(3,1);
    
    switch method
        case 'all'
            A1 = source(1:3,:);
            A2 = target(1:3,:);
            [~,W] = size(A1);
            [~,W2] = size(A2);
            x = 1:W;
        case 'sample'
            A1 = source(1:3,1:13:end);
            A2 = target(1:3,1:13:end);
            [~,W] = size(A1);
            [~,W2] = size(A2);
            x = 1:W;
        otherwise
            fprintf('error');
            return
    end
    
    err = 10;
    iter = 0;
    A2r = A2;

    while err > thresh
        iter = iter + 1;
        % phase 1
        idxes = zeros(1,W);
        tic;
        for i=x
            [~,idx] = min( sum( (A2-repmat(A1(:,i),1,W2)).^2 ) );
            idxes(i) = idx;
        end
        fprintf('%f   -',err); toc;

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

        A2 = Rout*A2r + repmat(tout,1,W2); % always with original for numeric stability
        err = mean( sum((A1 - A2(:,idxes) ).^2) );
        
        %pcshow(A1,10,A2);
        %pause(0.001);
    end
    fprintf('err=%f,  i=%d\n',err,iter);
end
