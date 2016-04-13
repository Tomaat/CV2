function pc = transform(pc,R,t)
    pc(1:3,:) = R*pc(1:3,:)+repmat(t,1,size(pc,2));
end