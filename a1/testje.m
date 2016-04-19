% load source.mat
% load target.mat
% 
% source = source + randn(size(source))*0.01;
% target = target + randn(size(target))*0.01;
% 
% param(1).t = 'all'; param(1).p = 0;
% param(2).t = 'sample'; param(2).p = 2;
% param(3).t = 'sample'; param(3).p = 17;
% param(4).t = 'sample'; param(4).p = 47;
% param(5).t = 'sample'; param(5).p = 91;
% param(6).t = 'rand1'; param(6).p = 0.8;
% param(7).t = 'rand1'; param(7).p = 0.5;
% param(8).t = 'rand1'; param(8).p = 0.1;
% param(9).t = 'rand1'; param(9).p = 0.01;
% param(10).t = 'randi'; param(10).p = 0.8;
% param(11).t = 'randi'; param(11).p = 0.5;
% param(12).t = 'randi'; param(12).p = 0.1;
% param(13).t = 'randi'; param(13).p = 0.01;
% 
% 
% errs = zeros(1,length(param));
% iters = zeros(1,length(param));
% 
% for i=1:length(param)
%     [~,~,e,it] = ICP(source,target,param(i).t,param(i).p,0.002,15);
%     errs(i) = e;
%     iters(i) = it;
% end

clear all
[pc1,t1] = stitch('non-iter',1,99,'sample',91,0.002,20);
[pc2,t2] = stitch('non-iter',2,99,'sample',91,0.002,20);
[pc3,t3] = stitch('non-iter',4,99,'sample',91,0.002,20);
[pc4,t4] = stitch('non-iter',10,99,'sample',91,0.002,20);