%Author: Ysbrand Galama, David van Erkelens

% Performs stitching with ICP algorithm over the data in ./data
% @arguments:
% - type: for iterative or not (use 'iter' for iterative, and 'non-iter' for non-iterative)
% - nth: the sample size. e.g use nth=5 for every fifth frame
% - mx: the frame to stop stitching, use 99 to use all.
% and to controll the ICP algorithm
% - method: method for subsampling, one of (default all)
%    - 'all': use every point
%    - 'sample': use every <param>th point
%    - 'rand1': sample at the beginning random with a chance p=<param>
%    - 'randi': sample every iteration random with a chance p=<param>
% - param: the parameter for the method (default 13 for uniform and 0.5 for random)
% - thresh: the threshold of the MSE where the program will finish (default 0.0012)
% - MAXITER: the maximum amount of iterations which overrules the error (default 20)
% @return
% - pc_out: a cell-array with every point-cloud mapped to the same orientation
% - ttoc: total amount of time for every stitch
function [pc_out,ttoc] = stitch(type,nth,mx,method,param,thresh,MAXITER)
    
    source = get(0);
    pc_tot = cell(1,length(nth:nth:mx));
    %pc_tot = [];
    p = 1;
    ttoc = 0;
    switch type
        case 'iter'
            for i=nth:nth:mx
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
            for i=nth:nth:mx
                target = get(i);
                tic;
                [R,t] = ICP(source,target,method,param,thresh,MAXITER);
                ttoc = ttoc + toc;
                toc;
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
