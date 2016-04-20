function [pc_out,ttoc] = stitch(type,nth,mx,method,param,thresh,MAXITER)
    
    X = [22:1:mx 0:1:19];
    source = get(X(1));
    pc_tot = cell(1,length(nth:nth:mx));
    %pc_tot = [];
    p = 1;
    ttoc = 0;
    switch type
        case 'iter'
            for i=X(nth:nth:end)
                target = get(i);
                tic;
                %s_datasample = datasample(source, size(target,2));
                
                idx = randperm(size(source, 2));
                [R,t] = ICP(source,target,method,param,thresh,MAXITER);
                
                %[R,t] = ICP(source(:,idx(1:min(size(source, 2), size(target, 2) * 3))),target,method,param,thresh,MAXITER);
                toc;
                source = [source transform(target,R,t)];
                pc_tot{p} = transform(target,R,t);
                p = 1 + p;
            end
        otherwise
            for i=X(nth:nth:end)
                target = get(i);
                tic;
                [R,t] = ICP(source,target,method,param,thresh,MAXITER);
                ttoc = ttoc + toc;
                toc;
                %subplot(2,1,1);
                %pcshowpair(source,target);
                %subplot(2,1,2);
                %pcshow2(12,pc_tot,source,transform(target,R,t));
                %pause;
                %source = transform(target,R,t);
                %pc_tot = [pc_tot source];
                %pc_tot{p} = source;
                pc_tot{p} = transform(target,R,t);
                p = 1 + p;
            end
    end
    
    pc_out = pc_tot;%pc_reduce(pc_tot,neir);
end

function x = get(n)
    x = trim_pc(readPcd(sprintf('./data/%010d.pcd',n)));
end

function pc_out = pc_reduce(pc_tot,neir)
    pc_out = pc_tot(:,1:neir:end);
end