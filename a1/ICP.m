function [Rout,tout] = ICP(source,target,method,param,thresh,MAXITER)
    if nargin < 6
        MAXITER = 20;
        if nargin < 5
            thresh = 0.0012;
            if nargin < 4
                if nargin < 3
                    method = 'all';
                end
                switch method
                    case {'all','sample'}
                        param = 13;
                    case {'rand1','randi'}
                        param = 0.5;
                end
            end
        end
    end
    Rout = eye(3);
    tout = zeros(3,1);
    
    switch method
        case 'all'
            getA1 = @() get(source(1:3,:));
            getA2 = @(R,t) get(transform(target(1:3,:), R,t));
        case 'sample'
            getA1 = @() get(source(1:3,1:param:end));
            getA2 = @(R,t) get(transform(target(1:3,1:param:end), R,t));
        case 'rand1'
            ix = frand(size(source,2),param); 
            iy = frand(size(target,2),param);
            getA1 = @() get(source(1:3,ix));
            getA2 = @(R,t) get(transform(target(1:3,iy),R,t));
        case 'randi'
            getA1 = @() get(source(1:3,frand(size(source,2),param)));
            getA2 = @(R,t) get(transform(target(1:3,frand(size(target,2),param)),R,t));
        otherwise
            fprintf('error');
            return
    end
    
    err = 10;
    iter = 0;

    while err > thresh && MAXITER > iter
        [A1,W] = getA1();
        [A2,W2] = getA2(Rout,tout);
        iter = iter + 1;
        % phase 1
        idxes = zeros(1,W);
        tic;
        for i=1:W
            [~,idx] = min( sum( (A2-repmat(A1(:,i),1,W2)).^2 ) );
            idxes(i) = idx;
        end
        fprintf('(%f)   ',err); toc;

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

        A2t = R'*A2 + repmat(t,1,W2); % always with original for numeric stability
        err = mean( sum((A1 - A2t(:,idxes) ).^2) );
        
        pcshowpair(A1,A2,37);
        plot3([A1(1,:);A2(1,idxes)],[A1(2,:);A2(2,idxes)],[A1(3,:);A2(3,idxes)]);
        pause;
        %hold on
        %pcshowpair(A1,A2,37);
        %scatter3(A1c(1),A1c(3),A1c(2),5,'m','fill');
        %scatter3(A2c(1),A2c(3),A2c(2),5,'r','fill');
        %hold off
        %pause;
        %pause(0.001);
    end
    fprintf('err=%f,  i=%d\n',err,iter);
end

function [X,W] = get(X)
    W = size(X,2);
end

function x = frand(N,p)
    x = randperm(N);
    x = x(1:round(N*p));
end

