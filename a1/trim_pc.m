function pc = trim_pc(pc,dist)
    if nargin < 2
        dist = 5;
    end
    
    if size(pc,2) == 4
        pc = pc';
    elseif size(pc,2) == 3
        pc = [pc';zeros(1,size(pc,1))];
    end
    
    ix = sum(pc(1:3,:).^2) > dist;
    pc(:,ix) = [];%NaN(size(pc(:,ix)));
end