function pc_out = combine(pc1,pc2,thresh)
    if nargin < 3
        thresh = 0.001;
    end

    W = size(pc1,2);
    
    idxes = zeros(1,W);
    ms = ones(1,W)*thresh*2;
    for i=1:W
        [m,idx] = min( sum( (pc2-repmat(pc1(:,i),1,size(pc2,2))).^2 ) );
        idxes(i) = idx;
        ms(i) = m;
    end
    pc2(:,idxes(ms < thresh)) = [];
    pc_out = [pc1 pc2];
end