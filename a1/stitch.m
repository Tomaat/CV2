function pc_out = stitch(nth,mx,neir,method,param,thresh,MAXITER)
    
    source = get(0);
    pc_tot = [];
    for i=1:nth:mx
        target = get(i);
        tic;
        [R,t] = ICP(source,target,method,param,thresh,MAXITER);
        toc;
        %subplot(2,1,1);
        %pcshowpair(source,target);
        %subplot(2,1,2);
        %pcshow2(12,pc_tot,source,transform(target,R,t));
        %pause;
        source = transform(target,R,t);
        pc_tot = [pc_tot source];
        
    end
    
    pc_out = pc_reduce(pc_tot,neir);
end

function x = get(n)
    x = trim_pc(readPcd(sprintf('./data/%010d.pcd',n)));
end

function pc_out = pc_reduce(pc_tot,neir)
    pc_out = pc_tot(:,1:neir:end);
end